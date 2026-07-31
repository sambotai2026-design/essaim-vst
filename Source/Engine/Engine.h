// L'ESSAIM — moteur looper ×8, portage 1:1 du prototype HTML (essaim-looper.html).
// Toutes les constantes, courbes et machines d'états reproduisent le proto à l'identique.
#pragma once
#include <juce_audio_basics/juce_audio_basics.h>
#include <juce_audio_processors/juce_audio_processors.h>
#include <juce_dsp/juce_dsp.h>
#include <array>
#include <atomic>
#include <functional>

namespace essaim {

constexpr int    kNumTracks   = 8;
constexpr double kMaxLoopSec  = 120.0;
constexpr int    kMarginFr    = 1536;   // avance d'armement (frames)
constexpr int    kPreRollFr   = 384;    // pré-roll auto-rec (~8 ms)
constexpr int    kRingLen     = 8192;   // ring de pré-roll
constexpr int    kWavePoints  = 256;    // paires min/max envoyées à l'UI
constexpr int    kXFadeFr     = 256;    // fondu enchaîné au point de bouclage (~5 ms)

enum class St { Empty, Wait, Pending, Rec, Play, Stop };

enum class Fx { Filter, Drive, Dly, Rvb, Pan, Trim };
inline const char* fxKey (Fx f)
{
    switch (f) { case Fx::Filter: return "filter"; case Fx::Drive: return "drive";
                 case Fx::Dly: return "dly"; case Fx::Rvb: return "rvb";
                 case Fx::Pan: return "pan"; case Fx::Trim: return "trim"; }
    return "filter";
}
inline Fx fxFromKey (const juce::String& s)
{
    if (s == "drive") return Fx::Drive;  if (s == "dly") return Fx::Dly;
    if (s == "rvb")   return Fx::Rvb;    if (s == "pan") return Fx::Pan;
    if (s == "trim")  return Fx::Trim;   return Fx::Filter;
}
inline int fxDefault (Fx f)
{
    switch (f) { case Fx::Filter: case Fx::Pan: case Fx::Trim: return 64; default: return 0; }
}

// filterFreqs() du proto — bipolaire LP / flat / HP
struct FilterFreqs { double lp, hp; bool flat; };
FilterFreqs filterFreqs (int v, double sr);

struct Grid { bool on = false; double lenS = 0, t0S = 0, barS = 0; };

// ---------- une piste ----------
struct Track
{
    // contrôle (protégé par le lock du moteur)
    St      st        = St::Empty;
    int     bars      = 4;          // 0 = MAN
    bool    closing   = false;
    bool    watchMain = false;      // écoute déclencheur (grille posée)
    juce::int64 startF = 0, stopF = 0;   // frames "musicales"
    juce::int64 anchorF = 0;             // ancre de phase (playT0)
    int     jobId     = -1;

    // assignations / valeurs
    std::array<Fx,  3> assign { Fx::Filter, Fx::Dly, Fx::Rvb };
    std::array<int, 6> fx     { 64, 0, 0, 0, 64, 64 };   // par effet (ordre enum)
    int     fadVal    = 102;        // ~0.8 comme le proto (--p:.8)

    // tampon de boucle (capture directe, audio thread écrit, alloué au arm)
    juce::AudioBuffer<float> buf;   // 2 canaux
    juce::int64 bufLen  = 0;        // frames valides (boucle) — 0 = vide
    juce::int64 capBase = 0;        // frame flux correspondant à buf[0] (startF + comp)
    juce::int64 capCount= 0;        // frames déjà capturées
    bool    hasBuf    = false;
    double  durS      = 0;

    // DSP
    juce::dsp::IIR::Filter<float> hpL, hpR, lpL, lpR;
    float   curLp = 18000.f, curHp = 12.f, tgtLp = 18000.f, tgtHp = 12.f;
    juce::SmoothedValue<float> trimG { 1.f }, volG { 0.f }, muteG { 1.f },
                               dSend { 0.f }, rSend { 0.f }, panV { 0.f }, inTap { 0.f };
    float   driveA = 0.f;           // 0..1

    // mesure / affichage
    std::atomic<float> vu { 0.f };
    std::atomic<float> pos { 0.f };  // 0..1 phase de lecture
    int     recCount  = 0;           // mesure courante pendant REC (pour l'UI)
};

// ---------- événements vers l'UI ----------
struct UiEvent
{
    enum Type { Toast, ToastWarn, Wave, GridPosed } type = Toast;
    int track = -1;
    juce::String msg;
    std::vector<float> wave;   // paires min,max ×kWavePoints
};

// ---------- moteur ----------
class Engine
{
public:
    Engine();

    void prepare (double sr, int maxBlock);
    void releaseResources();
    void process (juce::AudioBuffer<float>& io);   // in/out stéréo

    // ---- contrôle (thread message ; verrouillé en interne) ----
    void press   (int i);
    void clearTrack (int i);
    void clearAll();
    void stopAll();
    void setFader (int i, int v0to127);
    void setFxRow (int i, int row, int v0to127);
    void resetFxRow (int i, int row);
    void setAssign (int i, int row, Fx f);
    void setAssignRow (int row, Fx f);
    void setBars (int i, int bars);
    void stepBars (int i, int dir);                 // cycle MAN,1,2,4,8,16,32
    void setSel  (int i);
    void stepSel (int dir);
    void setSync (bool b);
    void setMonitor (bool b);
    void setAutoAdv (bool b);
    void setAutoRec (bool b);
    void setThresh (double pct01);                  // 0.01..0.5
    void setCompMs (double ms);
    void setMasterGain01 (double g);                // déjà courbé côté UI (pow 1.5 * 1.1)

    // tick UI/LED (thread message, ~30–60 Hz) : gère WAIT/PENDING promus, autorec, événements
    void uiTick();

    // ---- lecture d'état (thread message) ----
    struct Snapshot
    {
        struct T { St st; int bars; bool closing; bool hasBuf; int fad;
                   std::array<int,3> fxRow; std::array<int,3> asg;
                   float vu, pos; int recCount; double durS; };
        std::array<T, kNumTracks> tr;
        Grid grid; int sel; bool sync, mon, adv, arec;
        double thresh, compMs; float inVu, masterVu;
    };
    Snapshot snapshot();
    std::vector<UiEvent> drainEvents();

    int  fxValue (int i, Fx f);
    int  fxValueRow (int i, int row);
    Fx   assignOf (int i, int row) { juce::ScopedLock l (lock); return tr[(size_t) i].assign[(size_t) row]; }
    int  faderOf (int i)           { juce::ScopedLock l (lock); return tr[(size_t) i].fadVal; }
    int  selected()                { juce::ScopedLock l (lock); return sel; }
    bool gridOn()                  { juce::ScopedLock l (lock); return grid.on; }

    // état par piste pour les LED (thread message)
    struct LedView { St st; bool closing; bool lastBar; bool hasBuf; bool isSel; };
    std::array<LedView, kNumTracks> ledView();

    double sampleRate() const { return sr; }
    juce::int64 nowFrames() const { return gf.load(); }

    // sauvegarde/restauration (réglages, pas les boucles)
    juce::ValueTree toState() const;
    void fromState (const juce::ValueTree&);

    // session complète : réglages + boucles audio (format .essaim)
    bool saveSession (juce::OutputStream& os);
    bool loadSession (juce::InputStream& is);
    void pushToast (const juce::String& m, bool warn = false);

private:
    // ---- helpers (lock tenu) ----
    juce::int64 curF() const { return gf.load(); }
    juce::int64 compF() const { return (juce::int64) juce::roundToInt (compMs / 1000.0 * sr); }
    juce::int64 gridNextFrame (juce::int64 margin) const;
    double grooveDelayTime() const;
    void armRec (Track& t);
    void endRec (Track& t);
    void cancelRec (Track& t);
    void armWatch (Track& t);
    void cancelWatch (Track& t);
    void finishRec (int idx);            // onDone : boucle prête (thread message via uiTick)
    std::vector<float> waveOfLocked (Track& t);   // décimation min/max (lock tenu)
    void allocCapture (Track& t, juce::int64 frames);

    // ---- audio thread ----
    void runCaptureAndWatch (const float* inL, const float* inR, int n, juce::int64 b0);
    void renderTracks (const float* inL, const float* inR, juce::AudioBuffer<float>& out, int n);

    mutable juce::CriticalSection lock;
    std::array<Track, kNumTracks> tr;
    Grid   grid;
    int    sel = 0;
    bool   sync = true, monitor = false, autoAdv = false, autoRec = false;
    double thresh = 0.03, compMs = 0;
    int    jobSeq = 0;

    double sr = 48000.0;
    std::atomic<juce::int64> gf { 0 };          // compteur global de frames
    std::atomic<float> inPeak { 0.f }, inVuA { 0.f }, masterVuA { 0.f };
    std::atomic<float> masterGain { 0.8626f };   // pow(108/127,1.5)*1.1 — défaut du proto

    // ring pré-roll
    std::array<float, kRingLen> ring0 {}, ring1 {};
    int ringW = 0;

    // jobs terminés à finaliser côté message (index de piste), et watch déclenchés
    std::array<std::atomic<int>, kNumTracks> doneFlag;     // 1 = capture finie
    std::array<std::atomic<juce::int64>, kNumTracks> startedAt; // -1 sinon (watch déclenché)

    // bus delay
    juce::dsp::DelayLine<float, juce::dsp::DelayLineInterpolationTypes::Linear> dly { 0 };
    juce::dsp::IIR::Filter<float> dlyToneL, dlyToneR;
    juce::SmoothedValue<float> dlyTime { 0.38f };
    juce::AudioBuffer<float> dlyBusBuf, rvbBusBuf, scratch, inBuf;

    // bus reverb
    juce::dsp::Convolution conv;

    // limiteur
    juce::dsp::Compressor<float> limiter;

    // événements UI
    juce::CriticalSection evLock;
    std::vector<UiEvent> events;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (Engine)
};

} // namespace essaim
