#include "Engine.h"

namespace essaim {

// ---------- courbes du proto ----------
FilterFreqs filterFreqs (int v, double sr)
{
    const double maxLp = juce::jmin (18000.0, sr * 0.45);
    if (v >= 58 && v < 70) return { maxLp, 12.0, true };
    if (v < 58)            return { 70.0 * std::pow (maxLp / 70.0, v / 58.0), 12.0, false };
    const double u = (v - 69) / (127.0 - 69.0);
    return { maxLp, 12.0 * std::pow (6000.0 / 12.0, u), false };
}

static inline float faderGain (int v)   { return std::pow (v / 127.f, 1.8f); }
static inline float trimGain  (int v)   { return v <= 64 ? v / 64.f : 1.f + (v - 64) / 63.f; }
static inline float sendDly   (int v)   { return std::pow (v / 127.f, 2.f) * 0.9f; }
static inline float sendRvb   (int v)   { return std::pow (v / 127.f, 2.f) * 0.8f; }
static inline float panOf     (int v)   { return juce::jlimit (-1.f, 1.f, (v - 64) / 63.f); }

Engine::Engine()
{
    for (auto& d : doneFlag)  d.store (0);
    for (auto& s : startedAt) s.store (-1);
}

void Engine::prepare (double newSr, int maxBlock)
{
    juce::ScopedLock l (lock);
    sr = newSr;
    gf.store (0);
    ring0.fill (0.f); ring1.fill (0.f); ringW = 0;

    juce::dsp::ProcessSpec spec { sr, (juce::uint32) maxBlock, 2 };

    for (auto& t : tr)
    {
        const auto ff = filterFreqs (t.fx[(int) Fx::Filter], sr);
        t.curLp = t.tgtLp = (float) ff.lp;  t.curHp = t.tgtHp = (float) ff.hp;
        t.hpL.coefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass (sr, t.curHp, 0.71f);
        t.hpR.coefficients = t.hpL.coefficients;
        t.lpL.coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass  (sr, t.curLp, 0.71f);
        t.lpR.coefficients = t.lpL.coefficients;
        t.hpL.reset(); t.hpR.reset(); t.lpL.reset(); t.lpR.reset();
        t.trimG.reset (sr, 0.02);  t.trimG.setCurrentAndTargetValue (trimGain (t.fx[(int) Fx::Trim]));
        t.volG.reset  (sr, 0.015); t.volG.setCurrentAndTargetValue (faderGain (t.fadVal));
        t.muteG.reset (sr, 0.008); t.muteG.setCurrentAndTargetValue (t.st == St::Stop ? 0.f : 1.f);
        t.dSend.reset (sr, 0.02);  t.dSend.setCurrentAndTargetValue (sendDly (t.fx[(int) Fx::Dly]));
        t.rSend.reset (sr, 0.02);  t.rSend.setCurrentAndTargetValue (sendRvb (t.fx[(int) Fx::Rvb]));
        t.panV.reset  (sr, 0.02);  t.panV.setCurrentAndTargetValue (panOf (t.fx[(int) Fx::Pan]));
        t.inTap.reset (sr, 0.02);  t.inTap.setCurrentAndTargetValue (0.f);
        t.driveA = t.fx[(int) Fx::Drive] / 127.f;
    }

    dly.setMaximumDelayInSamples ((int) (5.0 * sr));
    dly.prepare (spec);
    dly.reset();
    dlyToneL.coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass (sr, 3800.0, 1.0f);
    dlyToneR.coefficients = dlyToneL.coefficients;
    dlyToneL.reset(); dlyToneR.reset();
    dlyTime.reset (sr, 0.05); dlyTime.setCurrentAndTargetValue (0.38f);

    // IR de reverb : bruit * (1 - i/len)^2.4 sur 2.6 s — identique au proto
    {
        const int irLen = (int) std::floor (sr * 2.6);
        juce::AudioBuffer<float> ir (2, irLen);
        juce::Random rng (0x50545221);
        for (int c = 0; c < 2; ++c)
        {
            auto* d = ir.getWritePointer (c);
            for (int i = 0; i < irLen; ++i)
                d[i] = (rng.nextFloat() * 2.f - 1.f) * std::pow (1.f - (float) i / (float) irLen, 2.4f);
        }
        conv.prepare (spec);
        conv.loadImpulseResponse (std::move (ir), sr,
                                  juce::dsp::Convolution::Stereo::yes,
                                  juce::dsp::Convolution::Trim::no,
                                  juce::dsp::Convolution::Normalise::no);
    }

    limiter.prepare (spec);
    limiter.setThreshold (-3.f);
    limiter.setRatio (20.f);
    limiter.setAttack (2.f);      // ms
    limiter.setRelease (120.f);   // ms

    dlyBusBuf.setSize (2, maxBlock);
    rvbBusBuf.setSize (2, maxBlock);
    scratch.setSize (2, maxBlock);
    inBuf.setSize (2, maxBlock);

    outRingL.assign ((size_t) kOutRing, 0.f);
    outRingR.assign ((size_t) kOutRing, 0.f);
    decIn.assign (8192, 0.f); decOut.assign (8192, 0.f);
    decAccI = decAccO = 0.f;
    aecD.store (-1); aecGL.store (0.f); aecGR.store (0.f); aecQ.store (0.f);
    aecSearching = false; aecBadTicks = 0;
}

void Engine::releaseResources() {}

// ---------- grille ----------
juce::int64 Engine::gridNextFrame (juce::int64 margin) const
{
    const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
    const auto t0F  = (juce::int64) juce::roundToInt (grid.t0S * sr);
    const auto f    = curF() + margin;
    const auto k    = (juce::int64) std::ceil ((double) (f - t0F) / (double) barF);
    return t0F + juce::jmax ((juce::int64) 1, k) * barF;
}

double Engine::grooveDelayTime() const
{
    if (! grid.on) return 0.38;
    double t = grid.barS > 0 ? grid.barS : grid.lenS;
    if (t <= 0) return 0.38;
    while (t > 0.75) t /= 2;
    while (t < 0.18) t *= 2;
    return t;
}

// ---------- allocation capture ----------
void Engine::allocCapture (Track& t, juce::int64 frames)
{
    const auto want = juce::jmin ((juce::int64) (kMaxLoopSec * sr) + kRingLen, frames + kXFadeFr);
    if (t.buf.getNumSamples() < (int) want)
        t.buf.setSize (2, (int) want, false, false, true);   // pas de clear : on ne lit
    // jamais au-delà de ce qui a été écrit (lecture ⊆ [0,bufLen), fondu ⊆ [L,capCount))
}

// ---------- actions (lock tenu par l'appelant public) ----------
void Engine::armRec (Track& t)
{
    t.recCount = 0; t.capCount = 0; t.closing = false;
    t.onsetTake = false;
    t.snapCorr = 0;
    t.jobId = ++jobSeq;
    if (sync && grid.on)
    {
        t.startF = gridNextFrame (kMarginFr);
        if (t.bars > 0)
        {
            const auto barF = (juce::int64) juce::roundToInt (grid.barS * sr);
            t.stopF = t.startF + (juce::int64) t.bars * barF;   // fin automatique, calée
        }
        else t.stopF = 0;
        allocCapture (t, (juce::int64) (kMaxLoopSec * sr));   // large : M modifiable en cours
        t.st = St::Pending;
    }
    else
    {
        t.startF = curF() + kMarginFr;
        t.stopF  = 0;
        allocCapture (t, (juce::int64) (kMaxLoopSec * sr));
        t.st = St::Rec;   // promotion visuelle immédiate (comme le proto hors grille)
    }
    t.capBase = t.startF + compF();
}

void Engine::endRec (Track& t)
{
    if (t.onsetTake && sync && grid.on)
    {
        const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
        const auto k = juce::jmax ((juce::int64) 1,
                (juce::int64) std::ceil ((double) (curF() + kMarginFr - t.startF) / (double) barF));
        if (t.bars > 0 && t.stopF != 0 && t.startF + k * barF >= t.stopF) { t.closing = true; return; }
        t.stopF = t.startF + k * barF;
        t.closing = true;
        return;
    }
    if (sync && grid.on)
    {
        const auto barF = (juce::int64) juce::roundToInt (grid.barS * sr);
        auto f = gridNextFrame (kMarginFr);
        if (f <= t.startF) f = t.startF + barF;
        if (t.bars > 0 && t.stopF != 0 && f >= t.stopF) { t.closing = true; return; }  // la fin auto arrive déjà
        t.stopF = f;
    }
    else
    {
        t.stopF = juce::jmax (curF() + kMarginFr, t.startF + (juce::int64) juce::roundToInt (0.12 * sr));
    }
    t.closing = true;
}

void Engine::cancelRec (Track& t)
{
    t.jobId = -1; t.capCount = 0; t.closing = false;
    t.st = St::Empty;
}

void Engine::armWatch (Track& t)
{
    t.capCount = 0; t.closing = false;
    t.jobId = ++jobSeq;
    t.gateOpen = false;    // bref silence requis (anti queue de phrase), ouverture forcée après 1,5 s
    t.armedF = curF();
    t.watchMain = false;   // détection audio sample-accurate dans TOUS les cas
    allocCapture (t, (juce::int64) (kMaxLoopSec * sr));
    t.st = St::Wait;
}

void Engine::cancelWatch (Track& t)
{
    t.jobId = -1; t.capCount = 0; t.watchMain = false;
    t.st = St::Empty;
}

// ---------- API contrôle ----------
void Engine::press (int i)
{
    juce::ScopedLock l (lock);
    auto& t = tr[(size_t) i];
    switch (t.st)
    {
        case St::Empty:   if (sel != i) { sel = i; } armRec (t); break;
        case St::Wait:    cancelWatch (t); if (sel != i) sel = i; armRec (t); break;   // forcer le départ
        case St::Pending: cancelRec (t); break;
        case St::Rec:     if (! t.closing) endRec (t); break;
        case St::Play:    t.muteG.setTargetValue (0.f); t.st = St::Stop; break;
        case St::Stop:    t.muteG.setTargetValue (1.f); t.st = St::Play; break;
    }
}

void Engine::clearTrack (int i)
{
    juce::ScopedLock l (lock);
    auto& t = tr[(size_t) i];
    if (t.st == St::Wait)                        cancelWatch (t);
    if (t.st == St::Rec || t.st == St::Pending)  cancelRec (t);
    t.hasBuf = false; t.bufLen = 0; t.durS = 0; t.closing = false;
    t.st = St::Empty;
    t.muteG.setTargetValue (1.f);
    t.pos.store (0.f);
    bool allEmpty = true;
    for (auto& x : tr) if (x.hasBuf || x.st != St::Empty) { allEmpty = false; break; }
    if (allEmpty)
    {
        grid = {}; gridFromBpm = false;
        if (bpm > 0) poseGridFromBpmLocked();
        dlyTime.setTargetValue ((float) grooveDelayTime());
    }
    UiEvent e; e.type = UiEvent::Wave; e.track = i;   // vague vide → efface le canvas
    juce::ScopedLock el (evLock); events.push_back (std::move (e));
}

void Engine::clearAll()
{
    for (int i = 0; i < kNumTracks; ++i) clearTrack (i);
    pushToast (juce::String::fromUTF8 ("Tout est effacé."));
}

void Engine::stopAll()
{
    juce::ScopedLock l (lock);
    for (auto& t : tr)
        if (t.st == St::Play) { t.muteG.setTargetValue (0.f); t.st = St::Stop; }
}

void Engine::playAll()
{
    juce::ScopedLock l (lock);
    for (auto& t : tr)
        if (t.st == St::Stop && t.hasBuf) { t.muteG.setTargetValue (1.f); t.st = St::Play; }
}

void Engine::setFader (int i, int v) { juce::ScopedLock l (lock); auto& t = tr[(size_t) i]; t.fadVal = juce::jlimit (0, 127, v); t.volG.setTargetValue (faderGain (t.fadVal)); }

void Engine::setFxRow (int i, int row, int v)
{
    juce::ScopedLock l (lock);
    auto& t = tr[(size_t) i];
    const Fx f = t.assign[(size_t) row];
    const int vv = juce::jlimit (0, 127, v);
    t.fx[(int) f] = vv;
    switch (f)
    {
        case Fx::Filter: { const auto ff = filterFreqs (vv, sr); t.tgtLp = (float) ff.lp; t.tgtHp = (float) ff.hp; break; }
        case Fx::Drive:  t.driveA = vv / 127.f; break;
        case Fx::Dly:    t.dSend.setTargetValue (sendDly (vv)); break;
        case Fx::Rvb:    t.rSend.setTargetValue (sendRvb (vv)); break;
        case Fx::Pan:    t.panV.setTargetValue (panOf (vv)); break;
        case Fx::Trim:   t.trimG.setTargetValue (trimGain (vv)); break;
    }
}

void Engine::resetFxRow (int i, int row)
{
    Fx f;
    { juce::ScopedLock l (lock); f = tr[(size_t) i].assign[(size_t) row]; }
    setFxRow (i, row, fxDefault (f));
}

void Engine::setAssign (int i, int row, Fx f) { juce::ScopedLock l (lock); tr[(size_t) i].assign[(size_t) row] = f; }
void Engine::setAssignRow (int row, Fx f)     { juce::ScopedLock l (lock); for (auto& t : tr) t.assign[(size_t) row] = f; }
void Engine::setBars (int i, int b)
{
    juce::ScopedLock l (lock);
    auto& t = tr[(size_t) i];
    t.bars = b;
    // prise calée en cours : recaler la fin automatique si possible
    if ((t.st == St::Pending || t.st == St::Rec) && ! t.closing
        && sync && grid.on && t.jobId > 0)
    {
        const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
        if (b > 0)
        {
            const auto ns = t.startF + (juce::int64) b * barF;
            if (ns > curF() + kMarginFr) t.stopF = ns;      // encore dans le futur : appliqué
        }
        else t.stopF = 0;                                    // MAN : fin manuelle
    }
}

void Engine::stepBars (int i, int dir)
{
    static const int steps[] { 0, 1, 2, 4, 8, 16, 32, 64 };
    juce::ScopedLock l (lock);
    auto& t = tr[(size_t) i];
    int idx = 0; for (int k = 0; k < 8; ++k) if (steps[k] == t.bars) idx = k;
    idx = juce::jlimit (0, 7, idx + dir);
    t.bars = steps[idx];
    if ((t.st == St::Pending || t.st == St::Rec) && ! t.closing
        && sync && grid.on && t.jobId > 0)
    {
        const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
        if (t.bars > 0)
        {
            const auto ns = t.startF + (juce::int64) t.bars * barF;
            if (ns > curF() + kMarginFr) t.stopF = ns;
        }
        else t.stopF = 0;
    }
    pushToast ("Piste " + juce::String (i + 1) + " : "
               + (t.bars > 0 ? juce::String (t.bars) + " mesure" + (t.bars > 1 ? "s" : "")
                             : juce::String ("fin manuelle")) + ".");
}

void Engine::setSel (int i)  { juce::ScopedLock l (lock); sel = ((i % kNumTracks) + kNumTracks) % kNumTracks; }
void Engine::stepSel (int d) { juce::ScopedLock l (lock); sel = ((sel + d) % kNumTracks + kNumTracks) % kNumTracks; }
void Engine::setSync (bool b)    { juce::ScopedLock l (lock); sync = b; }
void Engine::setMonitor (bool b) { juce::ScopedLock l (lock); monitor = b; }
void Engine::setAutoAdv (bool b) { juce::ScopedLock l (lock); autoAdv = b; }
void Engine::setAutoRec (bool b)
{
    juce::ScopedLock l (lock);
    autoRec = b;
    if (b) pushToast ("AUTOREC : la piste " + juce::String (sel + 1)
                      + juce::String::fromUTF8 (" démarre au premier son (seuil ")
                      + juce::String (juce::roundToInt (thresh * 100)) + " %).");
}
void Engine::setThresh (double v) { juce::ScopedLock l (lock); thresh = juce::jlimit (0.01, 0.5, v); }

void Engine::poseGridFromBpmLocked()
{
    grid.on   = true;
    grid.barS = 4.0 * 60.0 / bpm;          // 4/4
    grid.lenS = grid.barS;
    grid.t0S  = (double) curF() / sr;
    gridFromBpm = true;
    dlyTime.setTargetValue ((float) grooveDelayTime());
}

void Engine::setBpm (double b)
{
    juce::ScopedLock l (lock);
    bpm = b <= 0 ? 0.0 : juce::jlimit (30.0, 300.0, b);
    bool anyBuf = false;
    for (auto& t : tr) if (t.hasBuf || t.st == St::Rec || t.st == St::Pending) { anyBuf = true; break; }
    if (bpm > 0)
    {
        if (! anyBuf)
        {
            poseGridFromBpmLocked();
            pushToast (juce::String::fromUTF8 ("Tempo maître : ") + juce::String (bpm, 1)
                       + juce::String::fromUTF8 (" BPM — grille armée. Toutes les prises s'arrêtent pile aux mesures choisies."));
        }
        else
            pushToast (juce::String::fromUTF8 ("Tempo noté (") + juce::String (bpm, 1)
                       + juce::String::fromUTF8 (" BPM) — appliqué après CLEAR ALL."), true);
    }
    else if (gridFromBpm && ! anyBuf)
    {
        grid = {}; gridFromBpm = false;
        dlyTime.setTargetValue ((float) grooveDelayTime());
        pushToast (juce::String::fromUTF8 ("Tempo auto : la 1\u00e8re boucle posera la grille."));
    }
}

void Engine::setClick (bool on) { juce::ScopedLock l (lock); clickOn = on; }
void Engine::setCompMs (double v) { juce::ScopedLock l (lock); compMs = juce::jlimit (0.0, 300.0, v); }
void Engine::setMasterGain01 (double g) { masterGain.store ((float) g); }

// ---------- tick côté message : autorec, fins de capture, monitoring, repisse, anti-retour ----------
void Engine::uiTick()
{
    // fins de capture signalées par l'audio thread
    for (int i = 0; i < kNumTracks; ++i)
        if (doneFlag[(size_t) i].exchange (0) == 1)
            finishRec (i);

    bool wantCalib = false;

    {
        juce::ScopedLock l (lock);

        // départs watch signalés par l'audio thread
        for (int i = 0; i < kNumTracks; ++i)
        {
            const auto s2 = startedAt[(size_t) i].exchange (-1);
            if (s2 >= 0 && tr[(size_t) i].st == St::Wait)
                tr[(size_t) i].st = St::Rec;   // startF/capBase d\u00e9j\u00e0 pos\u00e9s par l'audio
        }

        // updateAutoWatch() — déclencheur SIMPLE : un seuil, un anti-queue court
        for (int i = 0; i < kNumTracks; ++i)
        {
            auto& t = tr[(size_t) i];
            if (autoRec && i == sel && t.st == St::Empty) { armWatch (t); continue; }
            if (t.st == St::Wait)
            {
                if (! autoRec || i != sel) { cancelWatch (t); continue; }
                // le déclenchement (gate + seuil + attaque) vit dans le thread audio
            }
            if (t.st == St::Rec && ! t.closing && t.stopF == 0
                && curF() - t.startF > (juce::int64) (kMaxLoopSec * sr))
                endRec (t);
            if (t.st == St::Rec && t.bars > 0 && grid.on && sync && t.stopF != 0)
            {
                const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
                t.recCount = (int) juce::jlimit ((juce::int64) 1, (juce::int64) t.bars,
                                                 (curF() - t.startF) / barF + 1);
            }
            else t.recCount = 0;
        }

        for (int i = 0; i < kNumTracks; ++i)
            tr[(size_t) i].inTap.setTargetValue (monitor && i == sel ? 0.9f : 0.f);

        dlyTime.setTargetValue ((float) grooveDelayTime());

        // ---- détection de repisse : PUREMENT INDICATIVE (sur l'entrée déjà nettoyée) ----
        lbIn[lbW] = inVuA.load(); lbOut[lbW] = masterVuA.load();
        lbW = (lbW + 1) % kLbWin; if (lbFill < kLbWin) ++lbFill;
        bool detected = false;
        if (! monitor && lbFill == kLbWin)
        {
            double mi = 0, mo = 0;
            for (int k = 0; k < kLbWin; ++k) { mi += lbIn[k]; mo += lbOut[k]; }
            mi /= kLbWin; mo /= kLbWin;
            if (mo > 0.02 && mi > 0.5 * mo)
            {
                double sio = 0, sii = 0, soo = 0;
                for (int k = 0; k < kLbWin; ++k)
                {
                    const double a = lbIn[k] - mi, b = lbOut[k] - mo;
                    sio += a * b; sii += a * a; soo += b * b;
                }
                const double denom = std::sqrt (sii * soo);
                if (denom > 1e-9 && sio / denom > 0.9) detected = true;
            }
        }
        lbHold = detected ? 90 : juce::jmax (0, lbHold - 1);
        loopback.store (lbHold > 0);
        if (lbHold > 0 && curF() - lastLbToastF > (juce::int64) (30.0 * sr))
        {
            lastLbToastF = curF();
            pushToast (juce::String::fromUTF8 ("(!) L'entr\u00e9e ressemble fort \u00e0 la sortie (repisse). "
                       "Si l'ANTI-RETOUR est actif, il se recalibre\u2026"), true);
        }

        // calibration ANTI-RETOUR : d\u00e8s qu'une sortie joue et que D est inconnu
        if (aecOn.load() && ! aecSearching && aecD.load() < 0 && masterVuA.load() > 0.05f
            && curF() - lastAecTryF > (juce::int64) (2.0 * sr))
        { aecSearching = true; wantCalib = true; lastAecTryF = curF(); }

        // qualit\u00e9 : si l'entr\u00e9e nettoy\u00e9e reste corr\u00e9l\u00e9e \u00e0 la sortie, recalibrer
        if (aecOn.load() && aecD.load() > 0 && loopback.load())
        {
            if (++aecBadTicks > 90) { aecBadTicks = 0; aecD.store (-1); }
        }
        else aecBadTicks = 0;
    }

    if (wantCalib)
    {
        aecEstimate();
        aecSearching = false;
    }
}

// ---------- calibration ANTI-RETOUR (thread message, hors lock audio) ----------
void Engine::aecEstimate()
{
    // instantan\u00e9s
    std::vector<float> di, dm;
    juce::int64 nowF;
    {
        juce::ScopedLock l (lock);
        di = decIn; dm = decOut; nowF = curF();
    }
    const int W = 2048;                                   // fen\u00eatre (d\u00e9cim\u00e9e) ~0,7 s
    const auto head = (size_t) ((((juce::uint64) nowF) >> 4) & 8191u);
    auto at = [&] (const std::vector<float>& v, long i) { return v[(size_t) (((long) head - 8 - i) & 8191)]; };
    double so2 = 1e-12;
    for (int i = 0; i < W; ++i) { const double o = at (dm, i); so2 += o * o; }
    if (so2 < 1e-4) { aecSearching = false; return; }     // pas assez de signal de sortie

    const int maxLag = juce::jmin (4000, (int) (1.2 * sr / 16.0));   // jusqu'\u00e0 ~1,2 s
    std::vector<double> score ((size_t) maxLag, 0.0);
    double bestR = 0;
    for (int lag = 2; lag < maxLag; ++lag)
    {
        double dot = 0, si2 = 1e-12;
        for (int i = 0; i < W; i += 2)
        {
            const double a = at (di, i), b = at (dm, i + lag);
            dot += a * b; si2 += a * a;
        }
        const double r = dot / std::sqrt (si2 * so2 * 0.5);
        score[(size_t) lag] = r;
        if (r > bestR) bestR = r;
    }
    // signaux p\u00e9riodiques (m\u00e9tronome, boucles) : des alias apparaissent \u00e0
    // vraiD\u00e9lai + k*p\u00e9riode. On collecte les premiers pics forts, puis on ne
    // garde que celui qui ANNULE r\u00e9ellement (test du r\u00e9sidu) — un alias corr\u00e8le
    // mais n'annule rien, il est donc rejet\u00e9.
    std::vector<int> cands;
    for (int lag = 2; lag < maxLag && (int) cands.size() < 4; ++lag)
        if (score[(size_t) lag] >= 0.85 * bestR)
        {
            if (cands.empty() || lag - cands.back() > 8) cands.push_back (lag);
            else if (score[(size_t) lag] > score[(size_t) cands.back()]) cands.back() = lag;
        }
    if (cands.empty() || bestR < 0.3)
    {
        pushToast (juce::String::fromUTF8 ("ANTI-RETOUR : calibration impossible pour l'instant (signal trop faible)."), true);
        return;
    }

    juce::int64 bestD = -1; double bestC = 0;
    {
        juce::ScopedLock l (lock);
        const int N = 4096;
        const auto endF = nowF - 64;
        for (int cLag : cands)
        {
            // affinage pleine r\u00e9solution autour de ce candidat
            juce::int64 dBest = -1; double cBest = 0;
            const auto d0 = (juce::int64) cLag * 16;
            for (juce::int64 d = juce::jmax ((juce::int64) 32, d0 - 48); d <= d0 + 48; ++d)
            {
                double dot = 0, si = 1e-12, so = 1e-12;
                for (int i = 0; i < N; i += 2)
                {
                    const auto f = endF - i;
                    const int  ri = (int) (((juce::int64) ringW + (f - nowF) + 16LL * kRingLen) % kRingLen);
                    const auto oi = (size_t) ((juce::uint64) (f - d) & (juce::uint64) (kOutRing - 1));
                    const double a = (ring0[(size_t) ri] + ring1[(size_t) ri]) * 0.5;
                    const double b = (outRingL[oi] + outRingR[oi]) * 0.5;
                    dot += a * b; si += a * a; so += b * b;
                }
                const double c = dot / std::sqrt (si * so);
                if (c > cBest) { cBest = c; dBest = d; }
            }
            if (dBest <= 0 || cBest < 0.4) continue;

            // gains par moindres carr\u00e9s + TEST DU R\u00c9SIDU sur ce candidat
            double eL = 0, eR = 0, oo = 1e-12, ein = 1e-12;
            const int N2 = 8192;
            for (int i = 0; i < N2; i += 2)
            {
                const auto f = nowF - 64 - i;
                const int  ri = (int) (((juce::int64) ringW + (f - nowF) + 16LL * kRingLen) % kRingLen);
                const auto oi = (size_t) ((juce::uint64) (f - dBest) & (juce::uint64) (kOutRing - 1));
                eL += (double) ring0[(size_t) ri] * outRingL[oi];
                eR += (double) ring1[(size_t) ri] * outRingR[oi];
                oo += (double) outRingL[oi] * outRingL[oi] + (double) outRingR[oi] * outRingR[oi];
                ein += (double) ring0[(size_t) ri] * ring0[(size_t) ri]
                     + (double) ring1[(size_t) ri] * ring1[(size_t) ri];
            }
            const double gl = juce::jlimit (0.0, 4.0, 2.0 * eL / oo);
            const double gr = juce::jlimit (0.0, 4.0, 2.0 * eR / oo);
            double eres = 1e-12;
            for (int i = 0; i < N2; i += 2)
            {
                const auto f = nowF - 64 - i;
                const int  ri = (int) (((juce::int64) ringW + (f - nowF) + 16LL * kRingLen) % kRingLen);
                const auto oi = (size_t) ((juce::uint64) (f - dBest) & (juce::uint64) (kOutRing - 1));
                const double rl = (double) ring0[(size_t) ri] - gl * outRingL[oi];
                const double rr = (double) ring1[(size_t) ri] - gr * outRingR[oi];
                eres += rl * rl + rr * rr;
            }
            if (eres / ein > 0.6) continue;   // n'annule pas assez : alias -> candidat suivant

            aecGL.store ((float) gl);
            aecGR.store ((float) gr);
            aecD.store (dBest);
            aecQ.store ((float) cBest);
            bestD = dBest; bestC = cBest;
            break;
        }
    }
    if (bestD > 0 && bestC > 0.4)
        pushToast (juce::String::fromUTF8 ("ANTI-RETOUR calibr\u00e9 : d\u00e9lai ")
                   + juce::String (bestD * 1000.0 / sr, 1) + " ms.");
    else
        pushToast (juce::String::fromUTF8 ("ANTI-RETOUR : calibration \u00e9chou\u00e9e, nouvel essai au prochain son."), true);
}

// ---------- fin d'enregistrement (thread message) ----------
void Engine::finishRec (int idx)
{
    std::vector<float> wave;
    juce::String toastMsg; bool warn = false; bool posedGrid = false; juce::String gridMsg;

    {
        juce::ScopedLock l (lock);
        auto& t = tr[(size_t) idx];
        const auto frames = juce::jmax ((juce::int64) 1, t.stopF + compF() - t.capBase);
        t.bufLen = juce::jmin (frames, (juce::int64) t.buf.getNumSamples());
        t.hasBuf = true;
        t.durS   = (double) t.bufLen / sr;
        t.anchorF = t.stopF + t.snapCorr;    // pose recal\u00e9e sur le temps (snap sans warp)
        t.jobId = -1; t.closing = false;

        // signal quasi nul ?
        float pk = 0.f;
        const auto* L = t.buf.getReadPointer (0);
        for (juce::int64 s = 0; s < t.bufLen; s += 97) pk = juce::jmax (pk, std::abs (L[(int) s]));
        if (pk < 0.0005f) { toastMsg = "Piste " + juce::String (idx + 1)
                            + juce::String::fromUTF8 (" : signal très faible — vérifie l'entrée / le gain."); warn = true; }

        if (! grid.on)
        {
            gridFromBpm = false;
            const int nb = t.bars > 0 ? t.bars : 1;
            const auto barFexact = juce::jmax ((juce::int64) 1, (t.bufLen + nb / 2) / nb);
            t.bufLen = juce::jmin ((juce::int64) t.buf.getNumSamples(), barFexact * nb);
            t.durS   = (double) t.bufLen / sr;
            const double bl = (double) barFexact / sr;
            grid = { true, t.durS, (double) t.anchorF / sr, bl };
            posedGrid = true;
            gridMsg = juce::String::fromUTF8 ("Grille posée : ") + juce::String (t.durS, 2) + " s"
                    + (t.bars > 0 ? " = " + juce::String (t.bars) + " mesures" : juce::String())
                    + " (piste " + juce::String (idx + 1) + ").";
            dlyTime.setTargetValue ((float) grooveDelayTime());
        }

        t.muteG.setTargetValue (1.f);
        t.st = St::Play;
        if (autoAdv) sel = (idx + 1) % kNumTracks;

        wave = waveOfLocked (t);
    }

    {
        juce::int64 sc = 0;
        { juce::ScopedLock l (lock); sc = tr[(size_t) idx].snapCorr; }
        if (std::abs (sc) > (juce::int64) (0.001 * sr))
            pushToast (juce::String::fromUTF8 ("Prise recal\u00e9e sur le temps : ")
                       + (sc > 0 ? "+" : "") + juce::String (sc * 1000.0 / sr, 0) + " ms.");
    }
    if (toastMsg.isNotEmpty()) pushToast (toastMsg, warn);
    if (posedGrid)             pushToast (gridMsg);
    UiEvent e; e.type = UiEvent::Wave; e.track = idx; e.wave = std::move (wave);
    juce::ScopedLock el (evLock); events.push_back (std::move (e));
}

std::vector<float> Engine::waveOfLocked (Track& t)
{
    std::vector<float> wave ((size_t) kWavePoints * 2, 0.f);
    if (! t.hasBuf || t.bufLen <= 0) return wave;
    const auto* L  = t.buf.getReadPointer (0);
    const auto* Rp = t.buf.getReadPointer (1);
    const auto step = juce::jmax ((juce::int64) 1, t.bufLen / kWavePoints);
    for (int x = 0; x < kWavePoints; ++x)
    {
        float mn = 1.f, mx = -1.f;
        const auto s0 = (juce::int64) x * step, s1 = juce::jmin (t.bufLen, s0 + step);
        for (juce::int64 sm = s0; sm < s1; sm += 4)
        {
            const float v = (L[(int) sm] + Rp[(int) sm]) * 0.5f;
            mn = juce::jmin (mn, v); mx = juce::jmax (mx, v);
        }
        if (mx < mn) { mn = 0; mx = 0; }
        wave[(size_t) x * 2]     = mn;
        wave[(size_t) x * 2 + 1] = mx;
    }
    return wave;
}

void Engine::pushToast (const juce::String& m, bool warn)
{
    UiEvent e; e.type = warn ? UiEvent::ToastWarn : UiEvent::Toast; e.msg = m;
    juce::ScopedLock el (evLock); events.push_back (std::move (e));
}

std::vector<UiEvent> Engine::drainEvents()
{
    juce::ScopedLock el (evLock);
    auto out = std::move (events); events.clear(); return out;
}

// ---------- snapshot ----------
Engine::Snapshot Engine::snapshot()
{
    Snapshot s;
    juce::ScopedLock l (lock);
    for (int i = 0; i < kNumTracks; ++i)
    {
        auto& t = tr[(size_t) i]; auto& o = s.tr[(size_t) i];
        o.st = t.st; o.bars = t.bars; o.closing = t.closing; o.hasBuf = t.hasBuf;
        o.fad = t.fadVal; o.vu = t.vu.load(); o.pos = t.pos.load();
        o.recCount = t.recCount; o.durS = t.durS;
        for (int r = 0; r < 3; ++r) { o.asg[(size_t) r] = (int) t.assign[(size_t) r];
                                      o.fxRow[(size_t) r] = t.fx[(int) t.assign[(size_t) r]]; }
    }
    s.grid = grid; s.sel = sel; s.sync = sync; s.mon = monitor; s.adv = autoAdv; s.arec = autoRec;
    s.loopback = loopback.load();
    s.bpm = bpm; s.click = clickOn; s.gridFromBpm = gridFromBpm;
    s.aec = aecOn.load();
    s.aecCal = aecD.load() > 0;
    s.aecMs = aecD.load() > 0 ? (double) aecD.load() * 1000.0 / sr : 0.0;
    s.thresh = thresh; s.compMs = compMs;
    s.inVu = inVuA.load(); s.masterVu = masterVuA.load();
    return s;
}

int Engine::fxValue (int i, Fx f)      { juce::ScopedLock l (lock); return tr[(size_t) i].fx[(int) f]; }
int Engine::fxValueRow (int i, int r)  { juce::ScopedLock l (lock); auto& t = tr[(size_t) i]; return t.fx[(int) t.assign[(size_t) r]]; }

std::array<Engine::LedView, kNumTracks> Engine::ledView()
{
    std::array<LedView, kNumTracks> out {};
    juce::ScopedLock l (lock);
    const auto barF = juce::jmax ((juce::int64) 1, (juce::int64) juce::roundToInt (grid.barS * sr));
    for (int i = 0; i < kNumTracks; ++i)
    {
        auto& t = tr[(size_t) i];
        bool lastBar = false;
        if (t.st == St::Rec && ! t.closing && t.bars > 0 && t.stopF != 0 && grid.on)
            lastBar = curF() > t.stopF - barF;
        out[(size_t) i] = { t.st, t.closing, lastBar, t.hasBuf, i == sel };
    }
    return out;
}

// ---------- sauvegarde (réglages) ----------
juce::ValueTree Engine::toState() const
{
    juce::ValueTree v ("ESSAIM");
    juce::ScopedLock l (lock);
    v.setProperty ("sync", sync, nullptr);   v.setProperty ("adv", autoAdv, nullptr);
    v.setProperty ("arec", autoRec, nullptr); v.setProperty ("thresh", thresh, nullptr);
    v.setProperty ("comp", compMs, nullptr);  v.setProperty ("sel", sel, nullptr);
    v.setProperty ("mon", monitor, nullptr);  v.setProperty ("mgain", (double) masterGain.load(), nullptr);
    v.setProperty ("aec", aecOn.load(), nullptr);
    v.setProperty ("bpm", bpm, nullptr);
    v.setProperty ("click", clickOn, nullptr);
    for (int i = 0; i < kNumTracks; ++i)
    {
        juce::ValueTree tv ("T"); auto& t = tr[(size_t) i];
        tv.setProperty ("bars", t.bars, nullptr);
        tv.setProperty ("fad", t.fadVal, nullptr);
        for (int r = 0; r < 3; ++r) tv.setProperty ("a" + juce::String (r), fxKey (t.assign[(size_t) r]), nullptr);
        for (int f = 0; f < 6; ++f) tv.setProperty ("f" + juce::String (f), t.fx[f], nullptr);
        v.addChild (tv, -1, nullptr);
    }
    return v;
}

void Engine::fromState (const juce::ValueTree& v)
{
    if (! v.hasType ("ESSAIM")) return;
    juce::ScopedLock l (lock);
    sync = (bool) v.getProperty ("sync", true);
    autoAdv = (bool) v.getProperty ("adv", false);
    autoRec = (bool) v.getProperty ("arec", false);
    thresh = (double) v.getProperty ("thresh", 0.03);
    compMs = (double) v.getProperty ("comp", 0.0);
    sel = (int) v.getProperty ("sel", 0);
    monitor = (bool) v.getProperty ("mon", false);
    masterGain.store ((float) (double) v.getProperty ("mgain", 0.8626));
    aecOn.store ((bool) v.getProperty ("aec", true));
    bpm = (double) v.getProperty ("bpm", 0.0);
    clickOn = (bool) v.getProperty ("click", false);
    for (int i = 0; i < juce::jmin (kNumTracks, v.getNumChildren()); ++i)
    {
        auto tv = v.getChild (i); auto& t = tr[(size_t) i];
        t.bars = (int) tv.getProperty ("bars", 4);
        t.fadVal = (int) tv.getProperty ("fad", 102);
        for (int r = 0; r < 3; ++r) t.assign[(size_t) r] = fxFromKey (tv.getProperty ("a" + juce::String (r), "filter").toString());
        for (int f = 0; f < 6; ++f) t.fx[f] = (int) tv.getProperty ("f" + juce::String (f), fxDefault ((Fx) f));
        // ré-application DSP
        const auto ff = filterFreqs (t.fx[(int) Fx::Filter], sr);
        t.tgtLp = (float) ff.lp; t.tgtHp = (float) ff.hp;
        t.driveA = t.fx[(int) Fx::Drive] / 127.f;
        t.dSend.setTargetValue (sendDly (t.fx[(int) Fx::Dly]));
        t.rSend.setTargetValue (sendRvb (t.fx[(int) Fx::Rvb]));
        t.panV.setTargetValue (panOf (t.fx[(int) Fx::Pan]));
        t.trimG.setTargetValue (trimGain (t.fx[(int) Fx::Trim]));
        t.volG.setTargetValue (faderGain (t.fadVal));
    }
}

// ---------- AUDIO THREAD ----------
void Engine::runCaptureAndWatch (const float* inL, const float* inR, int n, juce::int64 b0)
{
    const auto b1 = b0 + n;
    // ring de pré-roll
    for (int q = 0; q < n; ++q)
    {
        ring0[(size_t) ringW] = inL[q]; ring1[(size_t) ringW] = inR[q];
        ringW = (ringW + 1) % kRingLen;
    }
    for (auto& t : tr)
    {
        if (t.st == St::Wait && t.jobId > 0)
        {
            if (aecOn.load() && aecD.load() < 0 && masterVuA.load() > 0.03f)
                continue;   // sortie active mais anti-retour pas encore calibr\u00e9 : on attend (~2 s max)
            if (! t.gateOpen)
            {
                const bool quiet   = b0 - lastAboveF.load() > (juce::int64) (0.08 * sr);
                const bool timeout = b0 - t.armedF          > (juce::int64) (1.5  * sr);
                if (quiet || timeout) t.gateOpen = true;
                else continue;   // queue de la prise précédente : on attend (1,5 s max)
            }
            int hit = -1;
            const float th = (float) thresh;
            for (int q = 0; q < n; ++q)
                if (std::abs (inL[q]) >= th || std::abs (inR[q]) >= th) { hit = q; break; }
            if (hit >= 0)
            {
                const auto onset = juce::jmax ((juce::int64) 0, b0 + hit - kPreRollFr);
                t.watchMain = false;
                t.snapCorr  = 0;
                t.onsetTake = (sync && grid.on);
                juce::int64 capFrom = onset;               // d\u00e9but de capture (frames flux)

                if (sync && grid.on)
                {
                    const auto barFq  = juce::jmax ((juce::int64) 4,
                                          (juce::int64) juce::roundToInt (grid.barS * sr));
                    const auto beatF  = juce::jmax ((juce::int64) 1, barFq / 4);
                    const auto t0F    = (juce::int64) juce::roundToInt (grid.t0S * sr);
                    const auto attack = (b0 + hit) - compF();          // frame musicale de l'attaque
                    const auto rel    = attack - t0F;
                    const auto bStar  = t0F + juce::roundToIntAccurate ((double) rel / (double) barFq) * barFq;

                    if (attack >= bStar)
                    {
                        // EN RETARD (jusqu'\u00e0 une demi-mesure) : la r\u00e9gion est cal\u00e9e
                        // PILE sur la mesure vis\u00e9e — le moteur remonte le ring pour
                        // capturer depuis la barre, ton retard devient ton groove,
                        // les fronti\u00e8res de boucle sont parfaites.
                        t.startF  = bStar;
                        capFrom   = bStar + compF();
                        t.snapCorr = 0;
                    }
                    else
                    {
                        // EN AVANCE (lev\u00e9e) : capture depuis ton attaque, pose
                        // aimant\u00e9e sur le temps le plus proche (comportement valid\u00e9).
                        t.startF = onset - compF();
                        capFrom  = onset;
                        const auto snapped = t0F + ((attack - t0F + beatF / 2) / beatF) * beatF;
                        t.snapCorr = snapped - (t.startF + (juce::int64) kPreRollFr);
                    }

                    if (t.bars > 0)
                        t.stopF = t.startF + (juce::int64) t.bars * barFq;   // longueur = pile N mesures
                    else
                        t.stopF = 0;                                          // MAN : fermeture en mesures enti\u00e8res
                }
                else
                {
                    t.startF = onset - compF();
                    t.stopF  = 0;
                }

                // remplissage depuis le ring : [capFrom, b0) — pr\u00e9-roll et/ou remont\u00e9e \u00e0 la mesure
                t.capBase = juce::jmax ((juce::int64) 0, capFrom);
                t.capCount = 0;
                const auto cnt = (juce::int64) (b0 - t.capBase);
                if (cnt > 0 && cnt < (juce::int64) kRingLen)
                {
                    for (juce::int64 q = 0; q < cnt && t.capCount < t.buf.getNumSamples(); ++q)
                    {
                        const int pp = (int) (((juce::int64) ringW + (t.capBase + q - b1) + 32LL * kRingLen) % kRingLen);
                        t.buf.setSample (0, (int) t.capCount, ring0[(size_t) pp]);
                        t.buf.setSample (1, (int) t.capCount, ring1[(size_t) pp]);
                        ++t.capCount;
                    }
                }
                startedAt[(size_t) (&t - tr.data())].store (t.capBase);
                t.st = St::Rec;     // capture active d\u00e8s maintenant
                // pas de continue : la capture ci-dessous prend la suite du bloc courant
            }
            else
                continue;
        }

        // capture active (Pending programmé ou Rec)
        if ((t.st == St::Pending || t.st == St::Rec) && t.jobId > 0)
        {
            if (t.st == St::Pending && b1 > t.startF)
                t.st = St::Rec;   // l'enregistrement commence dans ce bloc

            auto blendHead = [this] (Track& tt)
            {
                // fondu enchaîné tête/queue : la fin capturée en trop (kXFadeFr frames)
                // est fondue dans le début — plus de clic au point de bouclage.
                const auto L = tt.stopF + compF() - tt.capBase;
                if (L <= kXFadeFr) return;
                auto* bl = tt.buf.getWritePointer (0);
                auto* br = tt.buf.getWritePointer (1);
                for (int j = 0; j < kXFadeFr && L + j < tt.capCount; ++j)
                {
                    const float w = std::sin (((float) (j + 1) / (kXFadeFr + 1)) * juce::MathConstants<float>::halfPi);
                    const float u = std::cos (((float) (j + 1) / (kXFadeFr + 1)) * juce::MathConstants<float>::halfPi);
                    bl[j] = bl[j] * w + bl[(int) L + j] * u;
                    br[j] = br[j] * w + br[(int) L + j] * u;
                }
            };

            const auto js = t.capBase;
            const auto je = t.stopF != 0 ? t.stopF + compF() + kXFadeFr : (juce::int64) 1 << 60;
            if (je <= b0)
            {
                // terminé avant ce bloc
                blendHead (t);
                t.jobId = -1;
                doneFlag[(size_t) (&t - tr.data())].store (1);
                continue;
            }
            if (js >= b1) continue;
            const int s  = (int) juce::jmax ((juce::int64) 0, js - b0);
            const int e  = (int) juce::jmin ((juce::int64) n, je - b0);
            for (int q = s; q < e && t.capCount < t.buf.getNumSamples(); ++q)
            {
                t.buf.setSample (0, (int) t.capCount, inL[q]);
                t.buf.setSample (1, (int) t.capCount, inR[q]);
                ++t.capCount;
            }
            if (je <= b1)
            {
                blendHead (t);
                t.jobId = -1;
                doneFlag[(size_t) (&t - tr.data())].store (1);
            }
        }
    }
}

void Engine::renderTracks (const float* inL, const float* inR, juce::AudioBuffer<float>& out, int n)
{
    auto* dL = dlyBusBuf.getWritePointer (0); auto* dR = dlyBusBuf.getWritePointer (1);
    auto* rL = rvbBusBuf.getWritePointer (0); auto* rR = rvbBusBuf.getWritePointer (1);
    juce::FloatVectorOperations::clear (dL, n); juce::FloatVectorOperations::clear (dR, n);
    juce::FloatVectorOperations::clear (rL, n); juce::FloatVectorOperations::clear (rR, n);
    auto* oL = out.getWritePointer (0); auto* oR = out.getWritePointer (1);

    const auto now0 = gf.load();

    for (auto& t : tr)
    {
        // lissage des filtres
        if (std::abs (t.curLp - t.tgtLp) > 1.f || std::abs (t.curHp - t.tgtHp) > 0.1f)
        {
            t.curLp += (t.tgtLp - t.curLp) * 0.25f;
            t.curHp += (t.tgtHp - t.curHp) * 0.25f;
            t.hpL.coefficients = juce::dsp::IIR::Coefficients<float>::makeHighPass (sr, juce::jmax (5.f, t.curHp), 0.71f);
            t.hpR.coefficients = t.hpL.coefficients;
            t.lpL.coefficients = juce::dsp::IIR::Coefficients<float>::makeLowPass  (sr, juce::jmin ((float) (sr * 0.49), t.curLp), 0.71f);
            t.lpR.coefficients = t.lpL.coefficients;
        }

        const bool playing = (t.st == St::Play || t.st == St::Stop) && t.hasBuf && t.bufLen > 0;
        const bool tapOn   = t.inTap.getCurrentValue() > 0.0005f || t.inTap.getTargetValue() > 0.0005f;
        // fin d'enregistrement : la lecture doit démarrer PILE à stopF, dans ce bloc,
        // sans attendre que le thread message passe la piste en PLAY (sinon : trou + clic)
        const bool recTail = (t.st == St::Rec && t.stopF != 0 && t.buf.getNumSamples() > 0
                              && now0 + n > t.stopF);
        const juce::int64 tailLen = recTail
            ? juce::jmax ((juce::int64) 1, juce::jmin ((juce::int64) t.buf.getNumSamples(), t.stopF - t.startF))
            : 0;
        if (! playing && ! tapOn && ! recTail)
        {
            t.trimG.skip (n); t.volG.skip (n); t.muteG.skip (n);
            t.dSend.skip (n); t.rSend.skip (n); t.panV.skip (n); t.inTap.skip (n);
            t.vu.store (t.vu.load() * 0.86f);
            continue;
        }

        const float k = 1.f + t.driveA * 40.f;
        const float tk = std::tanh (k);
        const bool doDrive = t.driveA > 0.001f;
        float pk = 0.f;

        const auto* bL = t.hasBuf ? t.buf.getReadPointer (0) : nullptr;
        const auto* bR = t.hasBuf ? t.buf.getReadPointer (1) : nullptr;
        juce::int64 rp = 0;
        if (playing)
            rp = ((now0 - t.anchorF) % t.bufLen + t.bufLen) % t.bufLen;

        for (int q = 0; q < n; ++q)
        {
            float xl = 0.f, xr = 0.f;
            if (playing)
            {
                xl = bL[(int) rp]; xr = bR[(int) rp];
                if (++rp >= t.bufLen) rp = 0;
            }
            else if (recTail)
            {
                const auto gfq = now0 + q;
                if (gfq >= t.stopF + t.snapCorr)
                {
                    const auto idx = (gfq - (t.stopF + t.snapCorr)) % tailLen;
                    if (idx < t.capCount)
                    {
                        xl = t.buf.getSample (0, (int) idx);
                        xr = t.buf.getSample (1, (int) idx);
                    }
                }
            }
            const float tap = t.inTap.getNextValue();
            xl += inL[q] * tap; xr += inR[q] * tap;

            const float tg = t.trimG.getNextValue();
            xl *= tg; xr *= tg;
            if (doDrive) { xl = std::tanh (k * xl) / tk; xr = std::tanh (k * xr) / tk; }
            xl = t.lpL.processSample (t.hpL.processSample (xl));
            xr = t.lpR.processSample (t.hpR.processSample (xr));
            // StereoPanner Web Audio (source stéréo) :
            //   p<=0 : x=p+1 ; L' = L + R·cos(xπ/2) ; R' = R·sin(xπ/2)
            //   p>0  : x=p   ; L' = L·cos(xπ/2)      ; R' = R + L·sin(xπ/2)
            const float p = t.panV.getNextValue();
            float yl, yr;
            if (p <= 0.f)
            {
                const float x = (p + 1.f) * juce::MathConstants<float>::halfPi;
                yl = xl + xr * std::cos (x);
                yr = xr * std::sin (x);
            }
            else
            {
                const float x = p * juce::MathConstants<float>::halfPi;
                yl = xl * std::cos (x);
                yr = xr + xl * std::sin (x);
            }
            const float vg = t.volG.getNextValue();
            const float mg = t.muteG.getNextValue();
            const float ol = yl * vg * mg, orr = yr * vg * mg;
            oL[q] += ol; oR[q] += orr;
            const float ds = t.dSend.getNextValue(), rs = t.rSend.getNextValue();
            dL[q] += ol * ds; dR[q] += orr * ds;
            rL[q] += ol * rs; rR[q] += orr * rs;
            const float a = juce::jmax (std::abs (ol), std::abs (orr));
            if (a > pk) pk = a;
        }
        t.vu.store (juce::jmax (pk, t.vu.load() * 0.86f));
        if (playing)
            t.pos.store ((float) ((double) (((now0 + n - t.anchorF) % t.bufLen + t.bufLen) % t.bufLen) / (double) t.bufLen));
    }
}

void Engine::process (juce::AudioBuffer<float>& io)
{
    const int n = io.getNumSamples();
    const auto b0 = gf.load();

    const float* inL = io.getReadPointer (0);
    const float* inR = io.getNumChannels() > 1 ? io.getReadPointer (1) : inL;

    // copie d'entrée (io sera écrasé) — tampon préalloué dans prepare()
    inBuf.copyFrom (0, 0, inL, n);
    inBuf.copyFrom (1, 0, inR, n);

    // ANTI-RETOUR : soustraire notre propre sortie (délai D, gains gL/gR calibrés)
    // AVANT toute mesure/capture — le moteur ne voit plus que la voix.
    if (aecOn.load() && ! outRingL.empty())
    {
        const auto D = aecD.load();
        if (D > 0)
        {
            auto* wl = inBuf.getWritePointer (0);
            auto* wr = inBuf.getWritePointer (1);
            const float gL0 = aecGL.load(), gR0 = aecGR.load();
            for (int q = 0; q < n; ++q)
            {
                const auto f = b0 + q - D;
                if (f >= 0)
                {
                    const auto ix = (size_t) (f & (juce::int64) (kOutRing - 1));
                    wl[q] -= gL0 * outRingL[ix];
                    wr[q] -= gR0 * outRingR[ix];
                }
            }
            // adaptation lente des gains (suit le master du FLX10)
            double eo = 0, eiL = 0, eiR = 0;
            for (int q = 0; q < n; q += 2)
            {
                const auto f = b0 + q - D; if (f < 0) continue;
                const auto ix = (size_t) (f & (juce::int64) (kOutRing - 1));
                const double oL2 = outRingL[ix], oR2 = outRingR[ix];
                eo  += oL2 * oL2 + oR2 * oR2;
                eiL += (double) wl[q] * oL2;
                eiR += (double) wr[q] * oR2;
            }
            // pas d'adaptation quand la sortie est faible ; pas minuscule et borné :
            // la voix par-dessus les boucles ne peut plus faire dériver le gain
            if (eo > (double) n * 2e-3)
            {
                const float dl = (float) juce::jlimit (-0.003, 0.003, 0.08 * eiL / eo);
                const float dr = (float) juce::jlimit (-0.003, 0.003, 0.08 * eiR / eo);
                aecGL.store (juce::jlimit (0.f, 4.f, gL0 + dl));
                aecGR.store (juce::jlimit (0.f, 4.f, gR0 + dr));
            }
        }
    }
    const float* icL = inBuf.getReadPointer (0);
    const float* icR = inBuf.getReadPointer (1);

    // décimation ÷16 de l'entrée nettoyée (indexée par frame absolue)
    for (int q = 0; q < n; ++q)
    {
        decAccI += (icL[q] + icR[q]) * 0.5f;
        if ((((juce::uint64) (b0 + q)) & 15u) == 15u)
        {
            decIn[(size_t) ((((juce::uint64) (b0 + q)) >> 4) & 8191u)] = decAccI * (1.f / 16.f);
            decAccI = 0.f;
        }
    }

    // pic d'entrée
    {
        float pk = 0.f;
        for (int q = 0; q < n; q += 4) pk = juce::jmax (pk, juce::jmax (std::abs (icL[q]), std::abs (icR[q])));
        inPeak.store (pk);
        inVuA.store (juce::jmax (pk, inVuA.load() * 0.86f));
    }

    juce::ScopedLock l (lock);   // les blocs sont courts ; l'UI ne tient le lock que brièvement

    runCaptureAndWatch (icL, icR, n, b0);
    if (inPeak.load() >= (float) thresh)
        lastAboveF.store (b0 + n);

    io.clear();
    renderTracks (icL, icR, io, n);

    // bus delay (feedback + tone), temps lissé
    {
        auto* dL = dlyBusBuf.getWritePointer (0); auto* dR = dlyBusBuf.getWritePointer (1);
        auto* oL = io.getWritePointer (0);        auto* oR = io.getWritePointer (1);
        for (int q = 0; q < n; ++q)
        {
            const float tm = dlyTime.getNextValue() * (float) sr;
            dly.setDelay (juce::jlimit (1.f, (float) (5.0 * sr) - 2.f, tm));
            const float wl = dly.popSample (0);
            const float wr = dly.popSample (1);
            const float tl = dlyToneL.processSample (wl);
            const float trn = dlyToneR.processSample (wr);
            dly.pushSample (0, dL[q] + tl * 0.42f);
            dly.pushSample (1, dR[q] + trn * 0.42f);
            oL[q] += tl * 0.9f; oR[q] += trn * 0.9f;
        }
    }

    // bus reverb (convolution IR générée)
    {
        juce::dsp::AudioBlock<float> rb (rvbBusBuf);
        auto sub = rb.getSubBlock (0, (size_t) n);
        juce::dsp::ProcessContextReplacing<float> cx (sub);
        conv.process (cx);
        io.addFrom (0, 0, rvbBusBuf, 0, 0, n, 0.9f);
        io.addFrom (1, 0, rvbBusBuf, 1, 0, n, 0.9f);
    }

    // master gain + limiteur + makeup (Web Audio applique un makeup auto)
    io.applyGain (masterGain.load());
    {
        juce::dsp::AudioBlock<float> mb (io);
        juce::dsp::ProcessContextReplacing<float> cx (mb);
        limiter.process (cx);
    }
    io.applyGain (1.25f);

    // métronome (CLICK) : bip calé sur la grille, accent au début de mesure
    if (clickOn && grid.on && grid.barS > 0)
    {
        auto* oL = io.getWritePointer (0); auto* oR = io.getWritePointer (1);
        const auto barF  = juce::jmax ((juce::int64) 8, (juce::int64) juce::roundToInt (grid.barS * sr));
        const auto beatF = juce::jmax ((juce::int64) 2, barF / 4);
        const auto t0F   = (juce::int64) juce::roundToInt (grid.t0S * sr);
        const int  lenF  = (int) (0.012 * sr);
        for (int q = 0; q < n; ++q)
        {
            const auto f  = b0 + q - t0F;
            if (f < 0) continue;
            const auto ib = f % beatF;
            if (ib < lenF)
            {
                const bool accent = (f % barF) < beatF;
                const float ph = (float) ib / (float) sr;
                const float env = 1.f - (float) ib / (float) lenF;
                const float v = std::sin (juce::MathConstants<float>::twoPi * (accent ? 1760.f : 880.f) * ph)
                                * env * env * (accent ? 0.5f : 0.32f);
                oL[q] += v; oR[q] += v;
            }
        }
    }
    {
        float pk = 0.f;
        auto* oL = io.getReadPointer (0); auto* oR = io.getReadPointer (1);
        for (int q = 0; q < n; q += 4) pk = juce::jmax (pk, juce::jmax (std::abs (oL[q]), std::abs (oR[q])));
        masterVuA.store (juce::jmax (pk, masterVuA.load() * 0.86f));
    }

    // référence ANTI-RETOUR : exactement ce qui part vers la carte, indexé par frame
    if (! outRingL.empty())
    {
        const auto* oL = io.getReadPointer (0); const auto* oR = io.getReadPointer (1);
        for (int q = 0; q < n; ++q)
        {
            const auto ix = (size_t) ((juce::uint64) (b0 + q) & (juce::uint64) (kOutRing - 1));
            outRingL[ix] = oL[q]; outRingR[ix] = oR[q];
            decAccO += (oL[q] + oR[q]) * 0.5f;
            if ((((juce::uint64) (b0 + q)) & 15u) == 15u)
            {
                decOut[(size_t) ((((juce::uint64) (b0 + q)) >> 4) & 8191u)] = decAccO * (1.f / 16.f);
                decAccO = 0.f;
            }
        }
    }

    gf.store (b0 + n);
}

// ---------- session complète (.essaim) : réglages + boucles ----------
// Format v1 : "ESSAIMS1" | int32 taille XML | XML UTF-8 | par piste avec boucle :
//   int64 frames | float32 L[frames] | float32 R[frames]
static const char* kSessionMagic = "ESSAIMS1";

bool Engine::saveSession (juce::OutputStream& os)
{
    juce::ValueTree v;
    std::array<juce::AudioBuffer<float>, kNumTracks> bufs;
    std::array<juce::int64, kNumTracks> lens {};
    {
        juce::ScopedLock l (lock);
        v = toState();
        v.setProperty ("gridOn",  grid.on,  nullptr);
        v.setProperty ("gridLen", grid.lenS, nullptr);
        v.setProperty ("gridBar", grid.barS, nullptr);
        v.setProperty ("srSaved", sr, nullptr);
        for (int i = 0; i < kNumTracks; ++i)
        {
            auto& t = tr[(size_t) i];
            auto tv = v.getChild (i);
            tv.setProperty ("hasBuf", t.hasBuf, nullptr);
            tv.setProperty ("frames", (juce::int64) (t.hasBuf ? t.bufLen : 0), nullptr);
            tv.setProperty ("dur", t.durS, nullptr);
            if (t.hasBuf && t.bufLen > 0)
            {
                lens[(size_t) i] = t.bufLen;
                bufs[(size_t) i].setSize (2, (int) t.bufLen);
                bufs[(size_t) i].copyFrom (0, 0, t.buf, 0, 0, (int) t.bufLen);
                bufs[(size_t) i].copyFrom (1, 0, t.buf, 1, 0, (int) t.bufLen);
            }
        }
    }

    const auto xml = v.toXmlString();
    os.write (kSessionMagic, 8);
    const auto utf8 = xml.toRawUTF8();
    const auto xmlLen = (juce::int32) strlen (utf8);
    os.writeInt (xmlLen);
    os.write (utf8, (size_t) xmlLen);
    for (int i = 0; i < kNumTracks; ++i)
    {
        if (lens[(size_t) i] <= 0) continue;
        os.writeInt64 (lens[(size_t) i]);
        os.write (bufs[(size_t) i].getReadPointer (0), (size_t) lens[(size_t) i] * sizeof (float));
        os.write (bufs[(size_t) i].getReadPointer (1), (size_t) lens[(size_t) i] * sizeof (float));
    }
    os.flush();
    return true;
}

bool Engine::loadSession (juce::InputStream& is)
{
    char magic[9] {};
    if (is.read (magic, 8) != 8 || strcmp (magic, kSessionMagic) != 0) return false;
    const auto xmlLen = is.readInt();
    if (xmlLen <= 0 || xmlLen > 4 * 1024 * 1024) return false;
    juce::MemoryBlock mb ((size_t) xmlLen + 1, true);
    if (is.read (mb.getData(), xmlLen) != xmlLen) return false;
    const auto v = juce::ValueTree::fromXml (juce::String::fromUTF8 ((const char*) mb.getData(), xmlLen));
    if (! v.hasType ("ESSAIM")) return false;

    // lecture des boucles HORS lock (le disque ne doit pas bloquer l'audio)
    std::array<juce::AudioBuffer<float>, kNumTracks> bufs;
    std::array<juce::int64, kNumTracks> lens {};
    for (int i = 0; i < juce::jmin (kNumTracks, v.getNumChildren()); ++i)
    {
        const auto tv = v.getChild (i);
        if (! (bool) tv.getProperty ("hasBuf", false)) continue;
        const auto frames = (juce::int64) tv.getProperty ("frames", 0);
        if (frames <= 0 || frames > (juce::int64) (kMaxLoopSec * 192000.0)) return false;
        if (is.readInt64() != frames) return false;
        const int bytes = (int) (frames * (juce::int64) sizeof (float));
        bufs[(size_t) i].setSize (2, (int) frames);
        if (is.read (bufs[(size_t) i].getWritePointer (0), bytes) != bytes) return false;
        if (is.read (bufs[(size_t) i].getWritePointer (1), bytes) != bytes) return false;
        lens[(size_t) i] = frames;
    }

    const double savedSr = (double) v.getProperty ("srSaved", sr);

    {
        juce::ScopedLock l (lock);
        for (auto& t : tr)
        {
            if (t.st == St::Wait) cancelWatch (t);
            if (t.st == St::Rec || t.st == St::Pending) cancelRec (t);
        }
        fromState (v);
        const auto A0 = curF() + kMarginFr;    // toutes les boucles re-phasées, têtes alignées
        for (int i = 0; i < kNumTracks; ++i)
        {
            auto& t = tr[(size_t) i];
            if (lens[(size_t) i] > 0)
            {
                t.buf = std::move (bufs[(size_t) i]);
                t.bufLen = lens[(size_t) i];
                t.hasBuf = true;
                t.durS = savedSr > 0 ? (double) t.bufLen / savedSr : (double) t.bufLen / sr;
                t.anchorF = A0;
                t.st = St::Stop;                          // muettes, en phase — PLAY pour lancer
                t.muteG.setCurrentAndTargetValue (0.f);
                t.pos.store (0.f);
            }
            else
            {
                t.hasBuf = false; t.bufLen = 0; t.durS = 0; t.st = St::Empty;
                t.muteG.setCurrentAndTargetValue (1.f);
                t.pos.store (0.f);
            }
            t.closing = false; t.jobId = -1; t.capCount = 0;
        }
        grid.on   = (bool) v.getProperty ("gridOn", false);
        grid.lenS = (double) v.getProperty ("gridLen", 0.0);
        grid.barS = (double) v.getProperty ("gridBar", 0.0);
        grid.t0S  = (double) A0 / sr;
        dlyTime.setTargetValue ((float) grooveDelayTime());
    }

    // formes d'onde + toast
    for (int i = 0; i < kNumTracks; ++i)
    {
        UiEvent e; e.type = UiEvent::Wave; e.track = i;
        {
            juce::ScopedLock l (lock);
            if (tr[(size_t) i].hasBuf) e.wave = waveOfLocked (tr[(size_t) i]);
        }
        juce::ScopedLock el (evLock); events.push_back (std::move (e));
    }
    pushToast (juce::String::fromUTF8 ("Session chargée — pistes en STOP, en phase. PLAY pour lancer."));
    return true;
}

} // namespace essaim
