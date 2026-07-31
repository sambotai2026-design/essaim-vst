// Launch Control XL — mapping et LED identiques au proto HTML.
#pragma once
#include <juce_audio_devices/juce_audio_devices.h>
#include "../Engine/Engine.h"
#include <map>

namespace essaim {

struct Led { enum : int { OFF = 12, RED_LO = 13, RED = 15, AMBER_LO = 29, AMBER = 63, GREEN_LO = 28, GREEN = 60 }; };

static constexpr int kKnobCCs[3][8] = {
    { 13, 14, 15, 16, 17, 18, 19, 20 },   // rangée A (Send A)
    { 29, 30, 31, 32, 33, 34, 35, 36 },   // rangée B (Send B)
    { 49, 50, 51, 52, 53, 54, 55, 56 },   // rangée C (Pan/Device)
};
static constexpr int kFaderCCs[8]  = { 77, 78, 79, 80, 81, 82, 83, 84 };
static constexpr int kFocusNotes[8]= { 41, 42, 43, 44, 57, 58, 59, 60 };
static constexpr int kCtrlNotes[8] = { 73, 74, 75, 76, 89, 90, 91, 92 };

class LCXL : private juce::MidiInputCallback,
             private juce::Timer
{
public:
    explicit LCXL (Engine& e) : engine (e) { startTimer (130); }   // blink 260 ms = 2 ticks
    ~LCXL() override { close(); }

    // sélection manuelle ("" = auto : premier périphérique contenant "Launch Control XL")
    void setInputId  (const juce::String& id) { wantIn = id;  reopen = true; }
    void setOutputId (const juce::String& id) { wantOut = id; reopen = true; }

    struct DeviceInfo { juce::String id, name; };
    std::vector<DeviceInfo> inputs()  const { std::vector<DeviceInfo> v; for (auto& d : juce::MidiInput::getAvailableDevices())  v.push_back ({ d.identifier, d.name }); return v; }
    std::vector<DeviceInfo> outputs() const { std::vector<DeviceInfo> v; for (auto& d : juce::MidiOutput::getAvailableDevices()) v.push_back ({ d.identifier, d.name }); return v; }
    juce::String currentInName()  const { return in  != nullptr ? in->getName()  : juce::String(); }
    juce::String currentInId()    const { return in  != nullptr ? inId  : juce::String(); }
    juce::String currentOutId()   const { return out != nullptr ? outId : juce::String(); }
    bool connected() const { return in != nullptr; }

    // valeurs "fantôme" (position physique quand le takeover attend) — -1 sinon
    int ghostKnob (int track, int row) const { juce::ScopedLock l (gl); auto it = ghost.find (key ("k", row, track)); return it == ghost.end() ? -1 : it->second; }
    int ghostFader (int track)         const { juce::ScopedLock l (gl); auto it = ghost.find (key ("f", 0,  track)); return it == ghost.end() ? -1 : it->second; }
    void dropTakeover (const juce::String& k) { juce::ScopedLock l (gl); to.erase (k); ghost.erase (k); }
    void dropTakeoverKnob (int track, int row) { dropTakeover (key ("k", row, track)); }
    void dropTakeoverFader (int track)         { dropTakeover (key ("f", 0, track)); }

private:
    static juce::String key (const char* p, int row, int i)
    {
        return p[0] == 'f' ? "f" + juce::String (i) : "k" + juce::String (row) + "_" + juce::String (i);
    }

    void close()
    {
        if (in)  { in->stop(); in.reset(); }
        out.reset();
        ledCache.clear();
    }

    void openIfNeeded()
    {
        if (! reopen && in != nullptr) return;
        reopen = false;
        close();

        auto ins = juce::MidiInput::getAvailableDevices();
        juce::MidiDeviceInfo pickIn;
        for (auto& d : ins)
            if ((wantIn.isNotEmpty() && d.identifier == wantIn)
                || (wantIn.isEmpty() && d.name.containsIgnoreCase ("Launch Control XL")))
                { pickIn = d; break; }
        if (pickIn.identifier.isNotEmpty())
        {
            in = juce::MidiInput::openDevice (pickIn.identifier, this);
            if (in) { in->start(); inId = pickIn.identifier; }
        }

        auto outs = juce::MidiOutput::getAvailableDevices();
        juce::MidiDeviceInfo pickOut;
        for (auto& d : outs)
            if ((wantOut.isNotEmpty() && d.identifier == wantOut)
                || (wantOut.isEmpty() && d.name.containsIgnoreCase ("Launch Control XL")))
                { pickOut = d; break; }
        if (pickOut.identifier.isNotEmpty())
        {
            out = juce::MidiOutput::openDevice (pickOut.identifier);
            if (out) { outId = pickOut.identifier; sweep(); }
        }
    }

    // ---- entrée MIDI (thread MIDI) ----
    void handleIncomingMidiMessage (juce::MidiInput* src, const juce::MidiMessage& m) override
    {
        // garde stricte : seul un Launch Control XL pilote le looper.
        // Un DDJ/autre contrôleur sélectionné par erreur est ignoré (ses CC
        // recouvrent les nôtres et mettraient le bazar dans les potards).
        if (src == nullptr || ! src->getName().containsIgnoreCase ("Launch Control XL"))
            return;
        const auto* raw = m.getRawData();
        if (m.getRawDataSize() < 3) return;
        const int st = raw[0], d1 = raw[1], d2 = raw[2];
        const int type = st & 0xF0;

        if (type == 0xB0)
        {
            ch = st & 0x0F;
            for (int i = 0; i < 8; ++i)
                if (kFaderCCs[i] == d1)
                { hwControl (key ("f", 0, i), d2, engine.faderOf (i), [this, i] (int v) { engine.setFader (i, v); }); return; }
            for (int r = 0; r < 3; ++r)
                for (int i = 0; i < 8; ++i)
                    if (kKnobCCs[r][i] == d1)
                    { hwControl (key ("k", r, i), d2, engine.fxValueRow (i, r), [this, i, r] (int v) { engine.setFxRow (i, r, v); }); return; }
            if (d1 >= 104 && d1 <= 107 && d2 > 0)   // flèches
            {
                if (d1 >= 106) engine.stepSel (d1 == 106 ? -1 : 1);
                else           engine.stepBars (engine.selected(), d1 == 104 ? 1 : -1);
                return;
            }
        }
        else if (type == 0x90 && d2 > 0)
        {
            ch = st & 0x0F;
            for (int i = 0; i < 8; ++i) if (kFocusNotes[i] == d1) { engine.press (i); return; }
            for (int i = 0; i < 8; ++i) if (kCtrlNotes[i]  == d1) { engine.clearTrack (i); return; }
        }
    }

    // soft-takeover — hwControl() du proto
    void hwControl (const juce::String& k, int v, int cur, std::function<void (int)> apply)
    {
        juce::ScopedLock l (gl);
        auto& s = to[k];
        if (s.eng) { apply (v); s.last = v; ghost.erase (k); return; }
        const bool near = std::abs (v - cur) <= 4;
        const bool crossed = s.hasLast && ((s.last - cur) * (v - cur) <= 0);
        if (near || crossed) { s.eng = true; apply (v); ghost.erase (k); }
        else                 ghost[k] = v;
        s.last = v; s.hasLast = true;
    }

    // ---- LED (timer, thread message) ----
    void timerCallback() override
    {
        openIfNeeded();
        blinkPhase = ! blinkPhase;
        if (out == nullptr) return;
        const auto views = engine.ledView();
        for (int i = 0; i < 8; ++i)
        {
            const auto& t = views[(size_t) i];
            int f = Led::OFF;
            switch (t.st)
            {
                case St::Wait:    f = blinkPhase ? Led::AMBER_LO : Led::OFF; break;
                case St::Pending: f = blinkPhase ? Led::AMBER    : Led::OFF; break;
                case St::Rec:     f = (t.closing || t.lastBar) ? (blinkPhase ? Led::RED : Led::RED_LO) : Led::RED; break;
                case St::Play:    f = Led::GREEN; break;
                case St::Stop:    f = Led::AMBER; break;
                case St::Empty:   f = Led::OFF;   break;
            }
            sendLed (kFocusNotes[i], f);
            sendLed (kCtrlNotes[i], t.isSel ? Led::AMBER : (t.hasBuf ? Led::RED_LO : Led::OFF));
        }
    }

    void sendLed (int note, int vel)
    {
        auto it = ledCache.find (note);
        if (it != ledCache.end() && it->second == vel) return;
        ledCache[note] = vel;
        out->sendMessageNow (juce::MidiMessage::noteOn (ch + 1, note, (juce::uint8) vel));
    }

    void sweep()
    {
        ledCache.clear();
        for (int i = 0; i < 8; ++i)
        {
            juce::Timer::callAfterDelay (i * 55, [this, i]
            {
                if (out == nullptr) return;
                sendLed (kFocusNotes[i], Led::AMBER);
                sendLed (kCtrlNotes[i], Led::AMBER_LO);
            });
        }
    }

    Engine& engine;
    std::unique_ptr<juce::MidiInput>  in;
    std::unique_ptr<juce::MidiOutput> out;
    juce::String inId, outId, wantIn, wantOut;
    bool reopen = true;
    int  ch = 8;               // défaut : template factory 1 (canal 8, 0-based)
    bool blinkPhase = false;
    std::map<int, int> ledCache;

    struct TO { bool eng = false; bool hasLast = false; int last = 0; };
    mutable juce::CriticalSection gl;
    std::map<juce::String, TO> to;
    std::map<juce::String, int> ghost;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (LCXL)
};

} // namespace essaim
