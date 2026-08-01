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
    if (wrapperType == wrapperType_Standalone)
    {
        // le wrapper standalone persiste cet état en base64 dans son fichier de
        // réglages à CHAQUE fermeture et le relit au lancement : on n'y met que
        // les réglages (léger). Les boucles passent par les fichiers .essaim.
        if (auto xml = engine.toState().createXml())
            copyXmlToBinary (*xml, dest);
        return;
    }
    // VST3 : le projet du DAW embarque la session complète (réglages + boucles)
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
