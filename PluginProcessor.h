#pragma once

#include <JuceHeader.h>
#include <mysofa.h>
#include <vector>

//==============================================================================
class AudioPluginAudioProcessor final : public juce::AudioProcessor,
                                        private juce::OSCReceiver::Listener<juce::OSCReceiver::RealtimeCallback>
{
public:
    //==============================================================================
    AudioPluginAudioProcessor();
    ~AudioPluginAudioProcessor() override;

    // OSC handling methods
    void oscMessageReceived(const juce::OSCMessage& message) override;
    void setOscPortID(int newID);
    int getOscPortID() { return osc_port_ID; }
    bool getOscPortConnected() { return osc_connected; }

    // Head tracking parameter getters/setters
    void setYaw(float yaw) { currentYaw = yaw; }
    void setPitch(float pitch) { currentPitch = pitch; }
    void setRoll(float roll) { currentRoll = roll; }
    float getYaw() const { return currentYaw; }
    float getPitch() const { return currentPitch; }
    float getRoll() const { return currentRoll; }
    bool getEnableHeadTracking() const { return enableHeadTracking; }
    void setEnableHeadTracking(bool enabled) { enableHeadTracking = enabled; }
    void setFlipYaw(bool flip) { flipYaw = flip; }
    void setFlipPitch(bool flip) { flipPitch = flip; }
    void setFlipRoll(bool flip) { flipRoll = flip; }
    bool getFlipYaw() const { return flipYaw; }
    bool getFlipPitch() const { return flipPitch; }
    bool getFlipRoll() const { return flipRoll; }
    bool getRPYflag() const { return useRollPitchYaw; }
    void setRPYflag(bool flag) { useRollPitchYaw = flag; }

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;
    using AudioProcessor::processBlock;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //---------------------------------------------------------------------------------
    // HRTF related methods
    bool loadHRTF(const juce::File& sofaFile);
    bool updateHRTFPosition(float x, float y, float z);
    bool updateHRTFPositionSpherical(float azimuth, float elevation, float distance);
    
    // Helper methods for coordinate conversion
    void cartesianToSpherical(float x, float y, float z, float& azimuth, float& elevation, float& distance);
    void sphericalToCartesian(float azimuth, float elevation, float distance, float& x, float& y, float& z);

    juce::AudioProcessorValueTreeState apvts;
    
private:
    // Head tracking parameters
    float currentYaw = 0.0f;
    float currentPitch = 0.0f;
    float currentRoll = 0.0f;
    bool enableHeadTracking = false;
    bool flipYaw = false;
    bool flipPitch = false;
    bool flipRoll = false;
    bool useRollPitchYaw = false;

    // Multi-band air absorption filter (more accurate physical model)
    juce::dsp::ProcessorDuplicator<juce::dsp::IIR::Filter<float>, juce::dsp::IIR::Coefficients<float>> highShelfFilter;
    juce::dsp::ProcessorDuplicator<juce::dsp::IIR::Filter<float>, juce::dsp::IIR::Coefficients<float>> highPassFilter;
    
    // OSC-related member variables
    juce::OSCReceiver osc;
    bool osc_connected = false;
    int osc_port_ID = 9000;  // Default OSC port

    // Apply head tracking rotation to the HRTF coordinates
    void applyHeadRotation();

    // For thread safety
    juce::CriticalSection processLock;

    // Timer for HRTF updates to avoid too frequent changes
    float lastHRTFUpdateTime = 0.0f;
    const float minTimeBetweenHRTFUpdates = 0.05f; // 50ms minimum between updates
        
    // For smooth parameter transitions
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> azimuthSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> elevationSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> distanceSmoothed;

    // Convolution processors for HRTF
    juce::dsp::Convolution convolutionL;
    juce::dsp::Convolution convolutionR;

    // HRTF data
    struct MYSOFA_EASY* hrtf = nullptr;
    int hrtfFilterLength = 0;
    std::vector<float> leftIR;
    std::vector<float> rightIR;
    float leftDelay = 0.0f;
    float rightDelay = 0.0f;
    
    // Current HRTF position (in Cartesian coordinates used by mysofa)
    float currentX = 0.0f;
    float currentY = -1.0f;  // Default: sound source on the left
    float currentZ = 0.0f;
    
    // Current spherical coordinates for UI
    float currentAzimuth = 270.0f;   // Default: 270 degrees (left side)
    float currentElevation = 0.0f;   // Default: 0 degrees (ear level)
    float currentDistance = 1.0f;    // Default: 1 meter

    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioPluginAudioProcessor)
};

juce::AudioProcessorValueTreeState::ParameterLayout parameters();