#pragma once

#include <JuceHeader.h>
#include <mysofa.h>
#include <vector>

// Forward declaration
class BRIRProcessor;

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
    
    // BRIR control methods
    void setEnableBRIR(bool shouldBeEnabled);
    bool getEnableBRIR() const;
    void setRoomDimensions(float length, float width, float height);
    void setReverbTime(float t60);
    void setSourcePosition(float x, float y, float z);
    void setListenerPosition(float x, float y, float z);
    void setWallAbsorption(float leftWall, float rightWall, float frontWall, 
                          float backWall, float ceiling, float floor);
    
    // BRIR parameter getters
    float getRoomLength() const;
    float getRoomWidth() const;
    float getRoomHeight() const;
    float getReverbTime() const;
    
    // Listener position getters
    float getListenerX() const { return listenerX; }
    float getListenerY() const { return listenerY; }
    float getListenerZ() const { return listenerZ; }

    // Raw source position getters (for visualization)
    float getRawAzimuthDegrees() const { return rawAzimuth; }
    float getRawElevationDegrees() const { return rawElevation; }
    float getRawSourceDistance() const { return rawDistance; }

    // Source room position getters (for visualization)
    float getSourceRoomX() const { return sourceRoomX; }
    float getSourceRoomY() const { return sourceRoomY; }
    float getSourceRoomZ() const { return sourceRoomZ; }

    // Calculate absolute source position in room
    void updateSourceRoomPosition();

    // Update listener position in the BRIR processor
    void updateListenerPosition();
    
    // Source position getters (for visualization)
    float getAzimuthDegrees() const { return currentAzimuth; }
    float getElevationDegrees() const { return currentElevation; }
    float getSourceDistance() const { return currentDistance; }

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
    
    // BRIR processing
    std::unique_ptr<BRIRProcessor> brirProcessor;
    bool enableBRIR = false;
    
    // Listener position (absolute coordinates in meters)
    float listenerX = 5.6f;  // Default: center of default room
    float listenerY = 4.6f;  // Default: center of default room
    float listenerZ = 1.5f;  // Default: as specified in the paper

    // Multi-band air absorption filter (more accurate physical model)
    juce::dsp::ProcessorDuplicator<juce::dsp::IIR::Filter<float>, juce::dsp::IIR::Coefficients<float>> highShelfFilter;
    juce::dsp::ProcessorDuplicator<juce::dsp::IIR::Filter<float>, juce::dsp::IIR::Coefficients<float>> highPassFilter;
    
    // OSC-related member variables
    juce::OSCReceiver osc;
    bool osc_connected = false;
    int osc_port_ID = 9000;  // Default OSC port

    // Apply head rotation to the HRTF coordinates
    void applyHeadRotation();

    // For thread safety
    juce::CriticalSection processLock;

    // Flag to control source position updates from HRTF parameters
    bool updateSourceFromHRTF = true; // Initialize to true for first run

    // Timer for HRTF updates to avoid too frequent changes
    float lastHRTFUpdateTime = 0.0f;
    const float minTimeBetweenHRTFUpdates = 0.05f; // 50ms minimum between updates
        
    // For smooth parameter transitions
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> azimuthSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> elevationSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> distanceSmoothed;

    // For smooth parameter transitions for listener position
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> listenerXSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> listenerYSmoothed;
    juce::SmoothedValue<float, juce::ValueSmoothingTypes::Linear> listenerZSmoothed;

    // Convolution processors for HRTF
    juce::dsp::Convolution convolutionL;
    juce::dsp::Convolution convolutionR;

    // Calculate relative HRTF position based on listener and source positions
    void updateRelativeHRTFPosition();
  
    // Pre-rotation source position (for visualization)
    float rawAzimuth = 0.0f;   // Azimuth before head rotation is applied
    float rawElevation = 0.0f; // Elevation before head rotation is applied
    float rawDistance = 1.0f;  // Distance before any processing

    // Absolute source position in room coordinates
    float sourceRoomX = 0.0f;
    float sourceRoomY = 0.0f;
    float sourceRoomZ = 0.0f;

    // Source position relative offsets from listener
    float sourceOffsetX = 0.0f;
    float sourceOffsetY = -1.0f;
    float sourceOffsetZ = 0.0f;

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