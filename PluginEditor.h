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

private:
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