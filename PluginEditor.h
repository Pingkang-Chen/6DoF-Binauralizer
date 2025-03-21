#pragma once

#include "PluginProcessor.h"

//==============================================================================
class AudioPluginAudioProcessorEditor final : public juce::AudioProcessorEditor,
                                             private juce::Timer
{
public:
    explicit AudioPluginAudioProcessorEditor (AudioPluginAudioProcessor&);
    ~AudioPluginAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;
    
    // Timer callback for updating UI based on head tracking data
    void timerCallback() override;
    
    // Method to update listener position slider ranges
    void updateListenerSliderRanges();

private:
    // Room view component for visualizing room and positions
    class RoomViewComponent : public juce::Component,
                              private juce::Timer
    {
    public:
        RoomViewComponent(AudioPluginAudioProcessor& p);
        
        void paint(juce::Graphics& g) override;
        void timerCallback() override;
        
    private:
        AudioPluginAudioProcessor& processor;

        // Source position in room coordinates
        float sourceRoomX = 5.6f;
        float sourceRoomY = 4.6f;
        float sourceRoomZ = 1.5f;
    
        // Track HRTF parameters to detect changes
        float lastAzimuth = 0.0f;
        float lastElevation = 0.0f;
        float lastDistance = 1.0f;
    
        // Last listener position (to detect changes)
        float lastListenerX = 5.6f;
        float lastListenerY = 4.6f;
        float lastListenerZ = 1.5f;
    
        bool sourceInitialized = false;

        JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RoomViewComponent)
    };
    
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    AudioPluginAudioProcessor& processorRef;
    
    // HRTF position sliders (using spherical coordinates)
    juce::Slider azimuthSlider;
    juce::Slider elevationSlider;
    juce::Slider distanceSlider;

    // Head tracking controls
    juce::GroupComponent headTrackingGroup;
    juce::ToggleButton enableHeadTrackingButton;
    juce::Label yawLabel;
    juce::Label pitchLabel;
    juce::Label rollLabel;
    juce::Label yprAddressLabel;
    juce::Slider yawSlider;
    juce::Slider pitchSlider;
    juce::Slider rollSlider;
    juce::ToggleButton flipYawButton;
    juce::ToggleButton flipPitchButton;
    juce::ToggleButton flipRollButton;
    juce::ToggleButton useRPYButton;
    juce::Label oscPortLabel;
    juce::TextEditor oscPortTextEditor;
    juce::TextButton connectButton;
        
    // Parameter attachments for head tracking parameters
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> enableHeadTrackingAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> flipYawAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> flipPitchAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> flipRollAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> useRPYAttachment;
    
    // BRIR UI components
    juce::GroupComponent brirGroup;
    juce::ToggleButton enableBRIRButton;
    juce::Slider roomLengthSlider;
    juce::Slider roomWidthSlider;
    juce::Slider roomHeightSlider;
    juce::Slider reverbTimeSlider;
    juce::Slider wallAbsorptionSlider;

    juce::Label roomLengthLabel;
    juce::Label roomWidthLabel;
    juce::Label roomHeightLabel;
    juce::Label reverbTimeLabel;
    juce::Label wallAbsorptionLabel;

    // Listener position controls
    juce::Slider listenerXSlider;
    juce::Slider listenerYSlider;
    juce::Slider listenerZSlider;
    juce::Label listenerXLabel;
    juce::Label listenerYLabel;
    juce::Label listenerZLabel;
    juce::Label listenerPositionHeading;
    juce::Label listenerCoordinateInfo;
    
    // Room visualization component
    std::unique_ptr<RoomViewComponent> roomView;

    // Add parameter attachments for BRIR controls
    std::unique_ptr<juce::AudioProcessorValueTreeState::ButtonAttachment> enableBRIRAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> roomLengthAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> roomWidthAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> roomHeightAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> reverbTimeAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> wallAbsorptionAttachment;
    
    // Parameter attachments for listener position
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> listenerXAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> listenerYAttachment;
    std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment> listenerZAttachment;
        
    // OSC connection handling
    void connectOSCButtonClicked();
    
    // Labels for the sliders
    juce::Label azimuthLabel;
    juce::Label elevationLabel;
    juce::Label distanceLabel;
    
    // Group components
    juce::GroupComponent hrtfGroup;
    
    // Status label for debugging
    juce::Label statusLabel;
    
    // Parameter attachments
    std::vector<std::unique_ptr<juce::AudioProcessorValueTreeState::SliderAttachment>> attachment;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (AudioPluginAudioProcessorEditor)
};