#include "PluginProcessor.h"
#include "PluginEditor.h"

EssaimProcessor::EssaimProcessor()
    : juce::AudioProcessor (BusesProperties()
                            .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                            .withOutput ("Output", juce::AudioChannelSet::stereo(), true))
{
    startTimerHz (60);   // uiTick : autorec / fins de capture / monitoring
}

EssaimProcessor::~EssaimProcessor() { stopTimer(); }

void EssaimProcessor::prepareToPlay (double sr, int block) { engine.prepare (sr, block); }
void EssaimProcessor::releaseResources()                   { engine.releaseResources(); }

bool EssaimProcessor::isBusesLayoutSupported (const BusesLayout& l) const
{
    return l.getMainOutputChannelSet() == juce::AudioChannelSet::stereo()
        && (l.getMainInputChannelSet() == juce::AudioChannelSet::stereo()
            || l.getMainInputChannelSet() == juce::AudioChannelSet::mono());
}

void EssaimProcessor::processBlock (juce::AudioBuffer<float>& buf, juce::MidiBuffer& midi)
{
    juce::ScopedNoDenormals nd;
    midi.clear();
    engine.process (buf);
}

juce::AudioProcessorEditor* EssaimProcessor::createEditor() { return new EssaimEditor (*this); }

void EssaimProcessor::getStateInformation (juce::MemoryBlock& dest)
{
    // le projet du DAW embarque la session complète : réglages + boucles audio
    juce::MemoryOutputStream mos (dest, false);
    engine.saveSession (mos);
}

void EssaimProcessor::setStateInformation (const void* data, int size)
{
    juce::MemoryInputStream mis (data, (size_t) size, false);
    if (! engine.loadSession (mis))
        if (auto xml = getXmlFromBinary (data, size))          // anciens projets (réglages seuls)
            engine.fromState (juce::ValueTree::fromXml (*xml));
}

juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter() { return new EssaimProcessor(); }
