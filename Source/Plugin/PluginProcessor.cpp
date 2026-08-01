#include "PluginProcessor.h"
#include "PluginEditor.h"

EssaimProcessor::BusesProperties EssaimProcessor::essaimBuses()
{
    // Standalone : 8 canaux par bus -> le panneau audio natif permet de cocher
    // PLUSIEURS paires d'entrée et de sortie en même temps.
    // VST3 : stéréo classique (ce que les DAW attendent).
    const bool sa = juce::PluginHostType::getPluginLoadedAs()
                        == juce::AudioProcessor::wrapperType_Standalone;
    const auto in  = sa ? juce::AudioChannelSet::discreteChannels (8) : juce::AudioChannelSet::stereo();
    const auto out = sa ? juce::AudioChannelSet::discreteChannels (8) : juce::AudioChannelSet::stereo();
    return BusesProperties()
             .withInput  ("Input",  in,  true)
             .withOutput ("Output", out, true);
}

EssaimProcessor::EssaimProcessor()
    : juce::AudioProcessor (essaimBuses())
{
    startTimerHz (60);   // uiTick : autorec / fins de capture / monitoring
}

EssaimProcessor::~EssaimProcessor() { stopTimer(); }

void EssaimProcessor::prepareToPlay (double sr, int block)
{
    engine.prepare (sr, block);
    stereoBuf.setSize (2, juce::jmax (16, block));
}
void EssaimProcessor::releaseResources()                   { engine.releaseResources(); }

bool EssaimProcessor::isBusesLayoutSupported (const BusesLayout& l) const
{
    const int in  = l.getMainInputChannelSet().size();
    const int out = l.getMainOutputChannelSet().size();
    return in >= 1 && in <= 8 && out >= 2 && out <= 8;
}

void EssaimProcessor::processBlock (juce::AudioBuffer<float>& buf, juce::MidiBuffer& midi)
{
    juce::ScopedNoDenormals nd;
    midi.clear();
    const int n    = buf.getNumSamples();
    const int nIn  = juce::jmin (8, getTotalNumInputChannels());
    const int nOut = juce::jmin (8, getTotalNumOutputChannels());

    if (nIn == 2 && nOut == 2)
    {
        engine.process (buf);            // chemin direct (VST3 / stéréo simple)
        return;
    }

    // somme des paires d'entrée actives -> stéréo moteur
    stereoBuf.clear (0, 0, n);
    stereoBuf.clear (1, 0, n);
    for (int c = 0; c < nIn; ++c)
        stereoBuf.addFrom (c & 1, 0, buf, c, 0, n);
    if (nIn == 1)
        stereoBuf.copyFrom (1, 0, stereoBuf, 0, 0, n);

    engine.process (stereoBuf);

    // master dupliqué sur chaque paire de sortie active
    for (int c = 0; c < nOut; ++c)
        buf.copyFrom (c, 0, stereoBuf, c & 1, 0, n);
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
