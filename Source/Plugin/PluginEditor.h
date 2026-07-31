#pragma once
#include <juce_gui_extra/juce_gui_extra.h>
#include "PluginProcessor.h"

class EssaimEditor : public juce::AudioProcessorEditor,
                     private juce::Timer
{
public:
    explicit EssaimEditor (EssaimProcessor&);
    ~EssaimEditor() override;

    void resized() override;

private:
    void timerCallback() override;      // push d'état 30 Hz + événements
    void handleUiEvent (const juce::var& payload);
    juce::String buildStateJson();
    void openAudioSettings();

    EssaimProcessor& proc;
    std::unique_ptr<juce::WebBrowserComponent> web;
    bool uiReady = false;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (EssaimEditor)
};
