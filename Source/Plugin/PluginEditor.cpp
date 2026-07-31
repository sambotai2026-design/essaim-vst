#include "PluginEditor.h"
#include "BinaryData.h"

#if JUCE_STANDALONE_APPLICATION
 #include <juce_audio_plugin_client/Standalone/juce_StandaloneFilterWindow.h>
#endif

using namespace juce;

static std::optional<WebBrowserComponent::Resource> serveUi (const String& path)
{
    const auto p = path == "/" ? String ("/index.html") : path;
    if (p == "/index.html")
    {
        const auto* data = BinaryData::essaim_ui_html;
        const auto  size = BinaryData::essaim_ui_htmlSize;
        std::vector<std::byte> bytes ((size_t) size);
        std::memcpy (bytes.data(), data, (size_t) size);
        return WebBrowserComponent::Resource { std::move (bytes), "text/html" };
    }
    return std::nullopt;
}

EssaimEditor::EssaimEditor (EssaimProcessor& p)
    : AudioProcessorEditor (p), proc (p)
{
    WebBrowserComponent::Options opts;
    opts = opts.withBackend (WebBrowserComponent::Options::Backend::defaultBackend)
               .withNativeIntegrationEnabled()
               .withResourceProvider (serveUi)
               .withEventListener ("ui", [this] (const var& v) { handleUiEvent (v); })
               .withKeepPageLoadedWhenBrowserIsHidden();
   #if JUCE_WINDOWS
    opts = opts.withBackend (WebBrowserComponent::Options::Backend::webview2)
               .withWinWebView2Options (WebBrowserComponent::Options::WinWebView2 {}
                    .withUserDataFolder (File::getSpecialLocation (File::tempDirectory)));
   #endif

    web = std::make_unique<WebBrowserComponent> (opts);
    addAndMakeVisible (*web);
    web->goToURL (WebBrowserComponent::getResourceProviderRoot());

    setResizable (true, true);
    setResizeLimits (980, 620, 2400, 1500);
    setSize (1240, 760);
    startTimerHz (30);
}

EssaimEditor::~EssaimEditor() { stopTimer(); }

void EssaimEditor::resized() { if (web) web->setBounds (getLocalBounds()); }

// ---------- JS → C++ ----------
void EssaimEditor::handleUiEvent (const var& v)
{
    const auto fn = v.getProperty ("fn", "").toString();
    auto& e = proc.engine;
    const int i   = (int) v.getProperty ("i", 0);
    const int r   = (int) v.getProperty ("r", 0);
    const int val = (int) v.getProperty ("v", 0);

    if      (fn == "ready")    { uiReady = true; }
    else if (fn == "press")    e.press (i);
    else if (fn == "clear")    e.clearTrack (i);
    else if (fn == "clearAll") e.clearAll();
    else if (fn == "stopAll")  e.stopAll();
    else if (fn == "fader")    { e.setFader (i, val); proc.lcxl.dropTakeoverFader (i); }
    else if (fn == "fx")       { e.setFxRow (i, r, val); proc.lcxl.dropTakeoverKnob (i, r); }
    else if (fn == "fxDbl")    { e.resetFxRow (i, r); proc.lcxl.dropTakeoverKnob (i, r); }
    else if (fn == "assign")   { e.setAssign (i, r, essaim::fxFromKey (v.getProperty ("eff", "filter").toString())); proc.lcxl.dropTakeoverKnob (i, r); }
    else if (fn == "assignRow"){ e.setAssignRow (r, essaim::fxFromKey (v.getProperty ("eff", "filter").toString()));
                                 for (int k = 0; k < essaim::kNumTracks; ++k) proc.lcxl.dropTakeoverKnob (k, r); }
    else if (fn == "bars")     e.setBars (i, val);
    else if (fn == "stepBars") e.stepBars (e.selected(), val);
    else if (fn == "sel")      e.setSel (i);
    else if (fn == "selStep")  e.stepSel (val);
    else if (fn == "sync")     e.setSync (val != 0);
    else if (fn == "mon")      e.setMonitor (val != 0);
    else if (fn == "adv")      e.setAutoAdv (val != 0);
    else if (fn == "arec")     e.setAutoRec (val != 0);
    else if (fn == "thresh")   e.setThresh ((double) val / 100.0);
    else if (fn == "comp")     e.setCompMs ((double) val);
    else if (fn == "master")   e.setMasterGain01 ((double) v.getProperty ("g", 0.85));
    else if (fn == "audio")    openAudioSettings();
    else if (fn == "midiIn")   proc.lcxl.setInputId  (v.getProperty ("id", "").toString());
    else if (fn == "midiOut")  proc.lcxl.setOutputId (v.getProperty ("id", "").toString());
}

void EssaimEditor::openAudioSettings()
{
   #if JUCE_STANDALONE_APPLICATION
    if (auto* holder = juce::StandalonePluginHolder::getInstance())
        holder->showAudioSettingsDialog();
   #endif
}

// ---------- C++ → JS ----------
static const char* stName (essaim::St s)
{
    using St = essaim::St;
    switch (s) { case St::Empty: return "EMPTY"; case St::Wait: return "WAIT";
                 case St::Pending: return "PENDING"; case St::Rec: return "REC";
                 case St::Play: return "PLAY"; case St::Stop: return "STOP"; }
    return "EMPTY";
}

juce::String EssaimEditor::buildStateJson()
{
    auto snap = proc.engine.snapshot();
    auto* root = new DynamicObject();
    Array<var> tracks;

    for (int i = 0; i < essaim::kNumTracks; ++i)
    {
        const auto& t = snap.tr[(size_t) i];
        auto* o = new DynamicObject();
        o->setProperty ("st", stName (t.st));
        o->setProperty ("bars", t.bars);
        o->setProperty ("closing", t.closing);
        o->setProperty ("hasBuf", t.hasBuf);
        o->setProperty ("fad", t.fad);
        o->setProperty ("vu", t.vu);
        o->setProperty ("pos", t.pos);
        o->setProperty ("rc", t.recCount);
        o->setProperty ("dur", t.durS);
        Array<var> fxv, asg, gk;
        for (int r = 0; r < 3; ++r)
        {
            fxv.add (t.fxRow[(size_t) r]);
            asg.add (String (essaim::fxKey ((essaim::Fx) t.asg[(size_t) r])));
            gk.add (proc.lcxl.ghostKnob (i, r));
        }
        o->setProperty ("fx", fxv);
        o->setProperty ("asg", asg);
        o->setProperty ("gk", gk);
        o->setProperty ("gf", proc.lcxl.ghostFader (i));
        tracks.add (var (o));
    }
    root->setProperty ("tr", tracks);

    auto* g = new DynamicObject();
    g->setProperty ("on", snap.grid.on);
    g->setProperty ("len", snap.grid.lenS);
    g->setProperty ("bar", snap.grid.barS);
    root->setProperty ("grid", var (g));

    root->setProperty ("sel", snap.sel);
    root->setProperty ("sr", proc.engine.sampleRate());
    root->setProperty ("sync", snap.sync);
    root->setProperty ("mon", snap.mon);
    root->setProperty ("adv", snap.adv);
    root->setProperty ("arec", snap.arec);
    root->setProperty ("thresh", juce::roundToInt (snap.thresh * 100));
    root->setProperty ("comp", snap.compMs);
    root->setProperty ("inVu", snap.inVu);
    root->setProperty ("masterVu", snap.masterVu);

    auto* midi = new DynamicObject();
    midi->setProperty ("ok", proc.lcxl.connected());
    midi->setProperty ("name", proc.lcxl.currentInName());
    midi->setProperty ("inId", proc.lcxl.currentInId());
    midi->setProperty ("outId", proc.lcxl.currentOutId());
    Array<var> ins, outs;
    for (auto& d : proc.lcxl.inputs())  { auto* x = new DynamicObject(); x->setProperty ("id", d.id); x->setProperty ("name", d.name); ins.add (var (x)); }
    for (auto& d : proc.lcxl.outputs()) { auto* x = new DynamicObject(); x->setProperty ("id", d.id); x->setProperty ("name", d.name); outs.add (var (x)); }
    midi->setProperty ("ins", ins);
    midi->setProperty ("outs", outs);
    root->setProperty ("midi", var (midi));

   #if JUCE_STANDALONE_APPLICATION
    root->setProperty ("standalone", true);
   #else
    root->setProperty ("standalone", false);
   #endif

    return JSON::toString (var (root), true);
}

void EssaimEditor::timerCallback()
{
    if (web == nullptr) return;

    // événements (toasts, waveforms)
    for (auto& ev : proc.engine.drainEvents())
    {
        auto* o = new DynamicObject();
        switch (ev.type)
        {
            case essaim::UiEvent::Toast:     o->setProperty ("toast", ev.msg); break;
            case essaim::UiEvent::ToastWarn: o->setProperty ("toast", ev.msg); o->setProperty ("warn", true); break;
            case essaim::UiEvent::GridPosed: o->setProperty ("toast", ev.msg); break;
            case essaim::UiEvent::Wave:
            {
                o->setProperty ("wave", ev.track);
                Array<var> pk; for (auto f : ev.wave) pk.add (f);
                o->setProperty ("pk", pk);
                break;
            }
        }
        web->evaluateJavascript ("window.__ev && window.__ev(" + JSON::toString (var (o), true) + ");");
    }

    web->evaluateJavascript ("window.__push && window.__push(" + buildStateJson() + ");");
}
