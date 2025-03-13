#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
AudioPluginAudioProcessorEditor::AudioPluginAudioProcessorEditor(AudioPluginAudioProcessor& p)
    : AudioProcessorEditor(&p), processorRef(p)
{
    // Set up HRTF sliders with spherical coordinates
    azimuthSlider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    azimuthSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 80, 20);
    azimuthSlider.setRotaryParameters(0.0f, 6.283185f, true); // 0 to 2π radians (0-360°)
    
    elevationSlider.setSliderStyle(juce::Slider::LinearVertical);
    elevationSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 80, 20);
    
    distanceSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    distanceSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    
    // HRTF controls - detaching labels to position them manually later
    azimuthLabel.setText("Azimuth", juce::dontSendNotification);
    // Don't attach to component here, will position manually
    
    elevationLabel.setText("Elevation", juce::dontSendNotification);
    // Don't attach to component here, will position manually
    
    distanceLabel.setText("Distance", juce::dontSendNotification);
    distanceLabel.attachToComponent(&distanceSlider, true);
    
    // Set up status label
    statusLabel.setText("HRTF Status: Using Spherical Coordinates", juce::dontSendNotification);
    statusLabel.setJustificationType(juce::Justification::centred);
    
    // Set up group components
    hrtfGroup.setText("HRTF Position Parameters (Spherical Coordinates)");
    
    // HRTF parameter attachments
    attachment.push_back(std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "hrtf_azimuth", azimuthSlider));
    attachment.push_back(std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "hrtf_elevation", elevationSlider));
    attachment.push_back(std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "hrtf_distance", distanceSlider));

    // Add HRTF components
    addAndMakeVisible(azimuthSlider);
    addAndMakeVisible(elevationSlider);
    addAndMakeVisible(distanceSlider);
    
    addAndMakeVisible(azimuthLabel);
    addAndMakeVisible(elevationLabel);
    addAndMakeVisible(distanceLabel);
    
    addAndMakeVisible(hrtfGroup);
    addAndMakeVisible(statusLabel);

    // Head tracking group
    headTrackingGroup.setText("Head Tracking");
    addAndMakeVisible(headTrackingGroup);
    
    // Enable head tracking checkbox
    enableHeadTrackingButton.setButtonText("Enable Head Tracking");
    enableHeadTrackingButton.setToggleState(processorRef.getEnableHeadTracking(), juce::dontSendNotification);
    enableHeadTrackingButton.onClick = [this]() { 
        processorRef.setEnableHeadTracking(enableHeadTrackingButton.getToggleState());
    };
    addAndMakeVisible(enableHeadTrackingButton);
    
    // YPR vs RPY radio button
    useRPYButton.setButtonText("Use RPY (Roll-Pitch-Yaw)");
    useRPYButton.setToggleState(processorRef.getRPYflag(), juce::dontSendNotification);
    useRPYButton.onClick = [this]() {
        processorRef.setRPYflag(useRPYButton.getToggleState());
    };
    addAndMakeVisible(useRPYButton);
    
    // Yaw control
    yawLabel.setText("Yaw \\ypr[0]", juce::dontSendNotification);
    yawLabel.setJustificationType(juce::Justification::centred);
    addAndMakeVisible(yawLabel);
    
    yawSlider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    yawSlider.setRange(-180.0, 180.0, 0.1);
    yawSlider.setValue(processorRef.getYaw(), juce::dontSendNotification);
    yawSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 80, 20);
    yawSlider.onValueChange = [this]() {
        processorRef.setYaw((float)yawSlider.getValue());
    };
    addAndMakeVisible(yawSlider);
    
    flipYawButton.setButtonText("+/-");
    flipYawButton.setToggleState(processorRef.getFlipYaw(), juce::dontSendNotification);
    flipYawButton.onClick = [this]() {
        processorRef.setFlipYaw(flipYawButton.getToggleState());
    };
    addAndMakeVisible(flipYawButton);
    
    // Pitch control
    pitchLabel.setText("Pitch \\ypr[1]", juce::dontSendNotification);
    pitchLabel.setJustificationType(juce::Justification::centred);
    addAndMakeVisible(pitchLabel);
    
    pitchSlider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    pitchSlider.setRange(-180.0, 180.0, 0.1);
    pitchSlider.setValue(processorRef.getPitch(), juce::dontSendNotification);
    pitchSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 80, 20);
    pitchSlider.onValueChange = [this]() {
        processorRef.setPitch((float)pitchSlider.getValue());
    };
    addAndMakeVisible(pitchSlider);
    
    flipPitchButton.setButtonText("+/-");
    flipPitchButton.setToggleState(processorRef.getFlipPitch(), juce::dontSendNotification);
    flipPitchButton.onClick = [this]() {
        processorRef.setFlipPitch(flipPitchButton.getToggleState());
    };
    addAndMakeVisible(flipPitchButton);
    
    // Roll control
    rollLabel.setText("Roll \\ypr[2]", juce::dontSendNotification);
    rollLabel.setJustificationType(juce::Justification::centred);
    addAndMakeVisible(rollLabel);
    
    rollSlider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
    rollSlider.setRange(-180.0, 180.0, 0.1);
    rollSlider.setValue(processorRef.getRoll(), juce::dontSendNotification);
    rollSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 80, 20);
    rollSlider.onValueChange = [this]() {
        processorRef.setRoll((float)rollSlider.getValue());
    };
    addAndMakeVisible(rollSlider);
    
    flipRollButton.setButtonText("+/-");
    flipRollButton.setToggleState(processorRef.getFlipRoll(), juce::dontSendNotification);
    flipRollButton.onClick = [this]() {
        processorRef.setFlipRoll(flipRollButton.getToggleState());
    };
    addAndMakeVisible(flipRollButton);
    
    // OSC port settings
    oscPortLabel.setText("OSC Port:", juce::dontSendNotification);
    addAndMakeVisible(oscPortLabel);
    
    oscPortTextEditor.setText(juce::String(processorRef.getOscPortID()), juce::dontSendNotification);
    addAndMakeVisible(oscPortTextEditor);
    
    connectButton.setButtonText("Connect");
    connectButton.onClick = [this]() { connectOSCButtonClicked(); };
    addAndMakeVisible(connectButton);
    
    // Start timer to update UI with current head tracking values
    startTimerHz(30); // Update UI at 30Hz
    
    // Make the window an appropriate size
    setSize(700, 500);
}

AudioPluginAudioProcessorEditor::~AudioPluginAudioProcessorEditor()
{
    stopTimer();
}

//==============================================================================
void AudioPluginAudioProcessorEditor::paint(juce::Graphics& g)
{
    // Fill the background
    g.fillAll(getLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId));
}

void AudioPluginAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced(10);
    
    // Status label at the bottom
    statusLabel.setBounds(area.removeFromBottom(30));
    
    // Leave a little space
    area.removeFromBottom(10);
    
    // Now divide into two parts: HRTF and head tracking
    auto halfHeight = area.getHeight() / 2;
    auto hrtfArea = area.removeFromTop(halfHeight);
    auto headTrackingArea = area;
    
    // Position the group components
    hrtfGroup.setBounds(hrtfArea);
    headTrackingGroup.setBounds(headTrackingArea);
    
    // Reduce the areas to account for the group borders
    hrtfArea.reduce(10, 25);
    headTrackingArea.reduce(10, 25);
    
    // Layout for HRTF controls with improved spacing
    auto hrtfLeft = hrtfArea.removeFromLeft(hrtfArea.getWidth() / 2);
    auto hrtfRight = hrtfArea;
    
    // Add padding at the top to move controls downward
    hrtfLeft.removeFromTop(20);  // Increased padding at the top
    hrtfRight.removeFromTop(20); // Increased padding at the top
    
    // Azimuth (rotary knob on left) - ENLARGED
    auto azimuthArea = hrtfLeft.removeFromTop(hrtfLeft.getHeight() / 2);
    // Make the azimuth knob larger by reducing the margins
    azimuthSlider.setBounds(azimuthArea.reduced(5, 5)); // Reduced margins to make it larger
    
    // Position the azimuth label above the knob
    auto azimuthLabelBounds = azimuthArea.reduced(5, 5); // Match the reduced margin
    azimuthLabelBounds = azimuthLabelBounds.removeFromTop(20);
    azimuthLabelBounds.setY(azimuthLabelBounds.getY() - 15); // Move it up a bit
    azimuthLabel.setBounds(azimuthLabelBounds);
    azimuthLabel.setJustificationType(juce::Justification::centred);
    
    // Distance (below azimuth)
    int sliderHeight = 30;
    int labelWidth = 120;
    distanceSlider.setBounds(hrtfLeft.removeFromTop(sliderHeight).withTrimmedLeft(labelWidth));
    
    // Elevation - adjust position to not be too close to the border
    auto elevationSliderBounds = hrtfRight.reduced(30, 10); // Increased horizontal reduction
    elevationSlider.setBounds(elevationSliderBounds);
    
    // Position the elevation label with more space
    auto elevationLabelBounds = juce::Rectangle<int>(
        hrtfRight.getCentreX() - 40, // Center X position minus half the width
        hrtfRight.getY() + 5,        // Top Y position plus a small offset
        80,                         // Width
        20                          // Height
    );
    elevationLabel.setBounds(elevationLabelBounds);
    elevationLabel.setJustificationType(juce::Justification::centred);
    
    // Position the head tracking controls
    
    // Enable head tracking toggle at the top
    enableHeadTrackingButton.setBounds(headTrackingArea.removeFromTop(30));
    headTrackingArea.removeFromTop(5);
    
    // RPY toggle and OSC port at the top
    auto topRow = headTrackingArea.removeFromTop(30);
    useRPYButton.setBounds(topRow.removeFromLeft(200));
    oscPortLabel.setBounds(topRow.removeFromLeft(70));
    oscPortTextEditor.setBounds(topRow.removeFromLeft(60));
    connectButton.setBounds(topRow.removeFromLeft(100));
    
    headTrackingArea.removeFromTop(10);
    
    // Position YPR controls in a row
    auto controlsArea = headTrackingArea.removeFromTop(120);
    auto sliderWidth = controlsArea.getWidth() / 3;
    
    // Yaw
    auto yawArea = controlsArea.removeFromLeft(sliderWidth);
    yawLabel.setBounds(yawArea.removeFromTop(20));
    yawSlider.setBounds(yawArea.removeFromTop(80));
    
    // Position the flip button to the right of the slider's text box
    auto yawButtonArea = yawArea.removeFromTop(20);
    auto yawButtonBounds = yawButtonArea.removeFromRight(40);
    flipYawButton.setBounds(yawButtonBounds);
    
    // Pitch
    auto pitchArea = controlsArea.removeFromLeft(sliderWidth);
    pitchLabel.setBounds(pitchArea.removeFromTop(20));
    pitchSlider.setBounds(pitchArea.removeFromTop(80));
    
    // Position the flip button to the right of the slider's text box
    auto pitchButtonArea = pitchArea.removeFromTop(20);
    auto pitchButtonBounds = pitchButtonArea.removeFromRight(40);
    flipPitchButton.setBounds(pitchButtonBounds);
    
    // Roll
    auto rollArea = controlsArea;
    rollLabel.setBounds(rollArea.removeFromTop(20));
    rollSlider.setBounds(rollArea.removeFromTop(80));
    
    // Position the flip button to the right of the slider's text box
    auto rollButtonArea = rollArea.removeFromTop(20);
    auto rollButtonBounds = rollButtonArea.removeFromRight(40);
    flipRollButton.setBounds(rollButtonBounds);
}

void AudioPluginAudioProcessorEditor::connectOSCButtonClicked()
{
    int port = oscPortTextEditor.getText().getIntValue();
    if (port >= 1024 && port <= 65535) {
        processorRef.setOscPortID(port);
        if (processorRef.getOscPortConnected()) {
            connectButton.setButtonText("Disconnect");
            statusLabel.setText("OSC Connected on port " + juce::String(port), juce::dontSendNotification);
        } else {
            connectButton.setButtonText("Connect");
            statusLabel.setText("OSC Connection failed on port " + juce::String(port), juce::dontSendNotification);
        }
    } else {
        statusLabel.setText("Invalid OSC port (use 1024-65535)", juce::dontSendNotification);
    }
}

void AudioPluginAudioProcessorEditor::timerCallback()
{
    // Only update if head tracking is enabled
    if (processorRef.getEnableHeadTracking())
    {
        // Update sliders without triggering callbacks (use dontSendNotification)
        yawSlider.setValue(processorRef.getYaw(), juce::dontSendNotification);
        pitchSlider.setValue(processorRef.getPitch(), juce::dontSendNotification);
        rollSlider.setValue(processorRef.getRoll(), juce::dontSendNotification);
        
        // Optionally update status label with current values for debugging
        if (processorRef.getOscPortConnected())
        {
            statusLabel.setText("OSC Connected on port " + juce::String(processorRef.getOscPortID()) 
                                + " - YPR: " + juce::String(processorRef.getYaw(), 1) + ", " 
                                + juce::String(processorRef.getPitch(), 1) + ", "
                                + juce::String(processorRef.getRoll(), 1),
                                juce::dontSendNotification);
        }
    }
}