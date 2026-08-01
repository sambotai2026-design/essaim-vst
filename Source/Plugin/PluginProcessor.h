#pragma once
#include <juce_audio_processors/juce_audio_processors.h>
#include "../Engine/Engine.h"
#include "../Midi/LCXL.h"

class EssaimProcessor : public juce::AudioProcessor,
                        private juce::Timer
{
public:
    EssaimProcessor();
    ~EssaimProcessor() override;

    void prepareToPlay (double sr, int block) override;
    void releaseResources() override;
    bool isBusesLayoutSupported (const BusesLayout&) const override;
    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override { return true; }

    const juce::String getName() const override { return "L'ESSAIM"; }
    bool acceptsMidi() const override { return true; }
    bool producesMidi() const override { return false; }
    bool isMidiEffect() const override { return false; }
    double getTailLengthSeconds() const override { return 3.0; }

    int getNumPrograms() override { return 1; }
    int getCurrentProgram() override { return 0; }
    void setCurrentProgram (int) override {}
    const juce::String getProgramName (int) override { return {}; }
    void changeProgramName (int, const juce::String&) override {}

    void getStateInformation (juce::MemoryBlock&) override;
    void setStateInformation (const void*, int) override;

    essaim::Engine engine;
    essaim::LCXL   lcxl { engine };

private:
    void timerCallback() override { engine.uiTick(); }
    juce::AudioBuffer<float> stereoBuf;   // somme d'entrée / source de duplication

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EssaimProcessor)
};
