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
    
    // Set up BRIR group
    brirGroup.setText("BRIR Environment Simulation");
    addAndMakeVisible(brirGroup);

    // Set up BRIR enable button
    enableBRIRButton.setButtonText("Enable BRIR Simulation");
    enableBRIRButton.setToggleState(processorRef.getEnableBRIR(), juce::dontSendNotification);
    addAndMakeVisible(enableBRIRButton);

    // Set up room dimension sliders
    roomLengthSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    roomLengthSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    addAndMakeVisible(roomLengthSlider);
    roomLengthLabel.setText("Room Length", juce::dontSendNotification);
    roomLengthLabel.attachToComponent(&roomLengthSlider, true);
    addAndMakeVisible(roomLengthLabel);

    roomWidthSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    roomWidthSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    addAndMakeVisible(roomWidthSlider);
    roomWidthLabel.setText("Room Width", juce::dontSendNotification);
    roomWidthLabel.attachToComponent(&roomWidthSlider, true);
    addAndMakeVisible(roomWidthLabel);

    roomHeightSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    roomHeightSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    addAndMakeVisible(roomHeightSlider);
    roomHeightLabel.setText("Room Height", juce::dontSendNotification);
    roomHeightLabel.attachToComponent(&roomHeightSlider, true);
    addAndMakeVisible(roomHeightLabel);

    // Set up reverb time slider
    reverbTimeSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    reverbTimeSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    addAndMakeVisible(reverbTimeSlider);
    reverbTimeLabel.setText("RT60 (s)", juce::dontSendNotification);
    reverbTimeLabel.attachToComponent(&reverbTimeSlider, true);
    addAndMakeVisible(reverbTimeLabel);

    // Set up wall absorption slider
    wallAbsorptionSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    wallAbsorptionSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 20);
    addAndMakeVisible(wallAbsorptionSlider);
    wallAbsorptionLabel.setText("Absorption", juce::dontSendNotification);
    wallAbsorptionLabel.attachToComponent(&wallAbsorptionSlider, true);
    addAndMakeVisible(wallAbsorptionLabel);


    // Create parameter attachments for BRIR controls
    enableBRIRAttachment = std::make_unique<juce::AudioProcessorValueTreeState::ButtonAttachment>(
        processorRef.apvts, "enable_brir", enableBRIRButton);
        
    roomLengthAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "room_length", roomLengthSlider);
        
    roomWidthAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "room_width", roomWidthSlider);
        
    roomHeightAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "room_height", roomHeightSlider);
        
    reverbTimeAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "reverb_time", reverbTimeSlider);
        
    wallAbsorptionAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "wall_absorption", wallAbsorptionSlider);
    
    // Set up listener position controls
    listenerPositionHeading.setText("Listener Position (meters)", juce::dontSendNotification);
    listenerPositionHeading.setFont(juce::Font(14.0f, juce::Font::bold));
    listenerPositionHeading.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(listenerPositionHeading);
    
    // Add coordinate system explanation
    listenerCoordinateInfo.setText("(0,0,0) = center of room", juce::dontSendNotification);
    listenerCoordinateInfo.setFont(juce::Font(12.0f, juce::Font::italic));
    listenerCoordinateInfo.setColour(juce::Label::textColourId, juce::Colours::grey);
    addAndMakeVisible(listenerCoordinateInfo);
    
    // Set up listener position sliders with centered style
    listenerXSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    listenerXSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    listenerXSlider.setDoubleClickReturnValue(true, 0.0f); // Double-click returns to center
    addAndMakeVisible(listenerXSlider);
    listenerXLabel.setText("Listener X", juce::dontSendNotification);
    listenerXLabel.attachToComponent(&listenerXSlider, true);
    addAndMakeVisible(listenerXLabel);
    
    listenerYSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    listenerYSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    listenerYSlider.setDoubleClickReturnValue(true, 0.0f); // Double-click returns to center
    addAndMakeVisible(listenerYSlider);
    listenerYLabel.setText("Listener Y", juce::dontSendNotification);
    listenerYLabel.attachToComponent(&listenerYSlider, true);
    addAndMakeVisible(listenerYLabel);
    
    listenerZSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    listenerZSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    listenerZSlider.setDoubleClickReturnValue(true, 0.0f); // Double-click returns to center
    addAndMakeVisible(listenerZSlider);
    listenerZLabel.setText("Listener Z", juce::dontSendNotification);
    listenerZLabel.attachToComponent(&listenerZSlider, true);
    addAndMakeVisible(listenerZLabel);
    
    // Create parameter attachments for listener position
    listenerXAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "listener_x", listenerXSlider);
        
    listenerYAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "listener_y", listenerYSlider);
        
    listenerZAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        processorRef.apvts, "listener_z", listenerZSlider);
    
    // Set up listeners to update slider ranges when room dimensions change
    roomLengthSlider.onValueChange = [this]() { updateListenerSliderRanges(); };
    roomWidthSlider.onValueChange = [this]() { updateListenerSliderRanges(); };
    roomHeightSlider.onValueChange = [this]() { updateListenerSliderRanges(); };
    
    // Set initial slider ranges
    updateListenerSliderRanges();
    
    // Create room view component
    roomView = std::make_unique<RoomViewComponent>(processorRef);
    addAndMakeVisible(*roomView);
    
    // Start timer to update UI with current head tracking values
    startTimerHz(30); // Update UI at 30Hz
    
    // Make the window an appropriate size to fit new BRIR controls
    setSize(950, 850); // Increased width to fit the room view
}

AudioPluginAudioProcessorEditor::~AudioPluginAudioProcessorEditor()
{
    stopTimer();
}

// === Fix for the PluginEditor.cpp updateListenerSliderRanges() method ===

void AudioPluginAudioProcessorEditor::updateListenerSliderRanges()
{
    // Get current room dimensions
    float roomLength = processorRef.getRoomLength();
    float roomWidth = processorRef.getRoomWidth();
    float roomHeight = processorRef.getRoomHeight();
    
    // Update listener position slider ranges to be symmetrical around zero
    // with a small margin from walls
    float halfLength = roomLength / 2.0f;
    float halfWidth = roomWidth / 2.0f;
    float halfHeight = roomHeight / 2.0f;
    
    // Set the ranges with a small margin
    float marginX = halfLength * 0.02f; // 2% margin
    float marginY = halfWidth * 0.02f;
    float marginZ = halfHeight * 0.02f;
    
    // Update sliders with symmetric ranges around zero
    listenerXSlider.setRange(-halfLength + marginX, halfLength - marginX, 0.01f);
    listenerYSlider.setRange(-halfWidth + marginY, halfWidth - marginY, 0.01f);
    listenerZSlider.setRange(-halfHeight + marginZ, halfHeight - marginZ, 0.01f);
    
    // Get current values
    float currentX = listenerXSlider.getValue();
    float currentY = listenerYSlider.getValue();
    float currentZ = listenerZSlider.getValue();
    
    // Ensure values stay within the new range
    currentX = juce::jlimit(-halfLength + marginX, halfLength - marginX, currentX);
    currentY = juce::jlimit(-halfWidth + marginY, halfWidth - marginY, currentY);
    currentZ = juce::jlimit(-halfHeight + marginZ, halfHeight - marginZ, currentZ);
    
    // Update the sliders with the clamped values
    listenerXSlider.setValue(currentX, juce::dontSendNotification);
    listenerYSlider.setValue(currentY, juce::dontSendNotification);
    listenerZSlider.setValue(currentZ, juce::dontSendNotification);
    
    // Important: Update the text box style to show the full range properly
    listenerXSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    listenerYSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    listenerZSlider.setTextBoxStyle(juce::Slider::TextBoxRight, false, 60, 20);
    
    // Update tooltip text to show available range
    listenerXSlider.setTooltip("X position: " + juce::String(-halfLength + marginX, 2) + "m to " 
                              + juce::String(halfLength - marginX, 2) + "m");
    listenerYSlider.setTooltip("Y position: " + juce::String(-halfWidth + marginY, 2) + "m to " 
                              + juce::String(halfWidth - marginY, 2) + "m");
    listenerZSlider.setTooltip("Z position: " + juce::String(-halfHeight + marginZ, 2) + "m to " 
                              + juce::String(halfHeight - marginZ, 2) + "m");
}

//==============================================================================
void AudioPluginAudioProcessorEditor::paint(juce::Graphics& g)
{
    // Fill the background
    g.fillAll(getLookAndFeel().findColour(juce::ResizableWindow::backgroundColourId));
}


// Update this code in the resized() method of PluginEditor.cpp:

void AudioPluginAudioProcessorEditor::resized()
{
    auto area = getLocalBounds().reduced(10);
    
    // Add a visual room representation to the right side
    auto visualArea = area.removeFromRight(250);
    
    // Status label at the bottom
    statusLabel.setBounds(area.removeFromBottom(30));
    
    // Leave a little space
    area.removeFromBottom(10);
    
    // Adjust the division of vertical space
    int availableHeight = area.getHeight();
    auto hrtfArea = area.removeFromTop(static_cast<int>(availableHeight * 0.28));    // 28% for HRTF
    auto headTrackingArea = area.removeFromTop(static_cast<int>(availableHeight * 0.30)); // 30% for head tracking
    auto brirArea = area; // remaining 42% for BRIR + listener controls
    
    // Position the group components
    hrtfGroup.setBounds(hrtfArea);
    headTrackingGroup.setBounds(headTrackingArea);
    brirGroup.setBounds(brirArea);
    
    // Reduce the areas to account for the group borders
    hrtfArea.reduce(10, 25);
    headTrackingArea.reduce(10, 25);
    brirArea.reduce(10, 25);
    
    // Layout for HRTF controls with improved spacing
    auto hrtfLeft = hrtfArea.removeFromLeft(hrtfArea.getWidth() / 2);
    auto hrtfRight = hrtfArea;
    
    // Add padding at the top to move controls downward
    hrtfLeft.removeFromTop(10);  // Reduced padding at the top
    hrtfRight.removeFromTop(10); // Reduced padding at the top
    
    // Size of YPR knobs for reference (to match)
    int knobSize = static_cast<int>(headTrackingArea.getHeight() * 0.45); // Reduced from 0.55 to 0.45
    
    // Azimuth - Make it the same size as YPR knobs
    azimuthLabel.setBounds(hrtfLeft.removeFromTop(20));
    azimuthLabel.setJustificationType(juce::Justification::centred);
    
    // Center the knob in its area both horizontally and vertically
    auto azimuthAreaHeight = hrtfLeft.getHeight() / 2; // Use half the height
    auto azimuthKnobArea = hrtfLeft.removeFromTop(azimuthAreaHeight);
    int centeredY = azimuthKnobArea.getCentreY() - (knobSize / 2);
    int centeredX = azimuthKnobArea.getCentreX() - (knobSize / 2);
    azimuthSlider.setBounds(centeredX, centeredY, knobSize, knobSize);
    
    // Distance (below azimuth)
    int sliderHeight = 30;
    int labelWidth = 120;
    hrtfLeft.removeFromTop(5); // Add a bit of spacing
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
    enableHeadTrackingButton.setBounds(headTrackingArea.removeFromTop(25));
    headTrackingArea.removeFromTop(5);
    
    // RPY toggle and OSC port at the top
    auto topRow = headTrackingArea.removeFromTop(25);
    useRPYButton.setBounds(topRow.removeFromLeft(200));
    oscPortLabel.setBounds(topRow.removeFromLeft(70));
    oscPortTextEditor.setBounds(topRow.removeFromLeft(60));
    connectButton.setBounds(topRow.removeFromLeft(100));
    
    headTrackingArea.removeFromTop(5);
    
    // Set up YPR group layout
    auto controlsArea = headTrackingArea;
    auto sliderWidth = controlsArea.getWidth() / 3;
    
    // YPR knob height (made consistent with azimuth)
    int yprKnobHeight = knobSize; // Use the same size as azimuth
    
    // Yaw group
    auto yawArea = controlsArea.removeFromLeft(sliderWidth);
    yawLabel.setBounds(yawArea.removeFromTop(20));
    yawLabel.setJustificationType(juce::Justification::centred);
    
    // Center the yaw knob in its area
    auto yawKnobArea = yawArea.removeFromTop(yprKnobHeight + 10);
    int yawCenteredX = yawKnobArea.getCentreX() - (yprKnobHeight / 2);
    int yawCenteredY = yawKnobArea.getCentreY() - (yprKnobHeight / 2);
    yawSlider.setBounds(yawCenteredX, yawCenteredY, yprKnobHeight, yprKnobHeight);
    
    // Position the flip button directly under the yaw knob
    int yawButtonX = yawSlider.getBounds().getCentreX() - 20; // 40/2 = 20 (half button width)
    int yawButtonY = yawSlider.getBounds().getBottom() + 5; // 5px below the knob
    flipYawButton.setBounds(yawButtonX, yawButtonY, 40, 20);
    
    // Pitch group
    auto pitchArea = controlsArea.removeFromLeft(sliderWidth);
    pitchLabel.setBounds(pitchArea.removeFromTop(20));
    pitchLabel.setJustificationType(juce::Justification::centred);
    
    // Center the pitch knob in its area
    auto pitchKnobArea = pitchArea.removeFromTop(yprKnobHeight + 10);
    int pitchCenteredX = pitchKnobArea.getCentreX() - (yprKnobHeight / 2);
    int pitchCenteredY = pitchKnobArea.getCentreY() - (yprKnobHeight / 2);
    pitchSlider.setBounds(pitchCenteredX, pitchCenteredY, yprKnobHeight, yprKnobHeight);
    
    // Position the flip button directly under the pitch knob
    int pitchButtonX = pitchSlider.getBounds().getCentreX() - 20;
    int pitchButtonY = pitchSlider.getBounds().getBottom() + 5;
    flipPitchButton.setBounds(pitchButtonX, pitchButtonY, 40, 20);
    
    // Roll group
    auto rollArea = controlsArea;
    rollLabel.setBounds(rollArea.removeFromTop(20));
    rollLabel.setJustificationType(juce::Justification::centred);
    
    // Center the roll knob in its area
    auto rollKnobArea = rollArea.removeFromTop(yprKnobHeight + 10);
    int rollCenteredX = rollKnobArea.getCentreX() - (yprKnobHeight / 2);
    int rollCenteredY = rollKnobArea.getCentreY() - (yprKnobHeight / 2);
    rollSlider.setBounds(rollCenteredX, rollCenteredY, yprKnobHeight, yprKnobHeight);
    
    // Position the flip button directly under the roll knob
    int rollButtonX = rollSlider.getBounds().getCentreX() - 20;
    int rollButtonY = rollSlider.getBounds().getBottom() + 5;
    flipRollButton.setBounds(rollButtonX, rollButtonY, 40, 20);
    
    // Position BRIR controls - use less space per control
    int brirSliderHeight = 22;  // Slightly smaller height
    int brirLabelWidth = 100;
    
    // Position BRIR controls with more compact spacing
    enableBRIRButton.setBounds(brirArea.removeFromTop(22));
    brirArea.removeFromTop(4);
    
    // Position room dimension sliders with less spacing
    roomLengthSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(4);
    roomWidthSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(4);
    roomHeightSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(6);

    // Position reverb time and absorption sliders with less spacing
    reverbTimeSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(4);
    wallAbsorptionSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(10);
    
    // Position listener position controls
    // Position the section heading
    listenerPositionHeading.setBounds(brirArea.removeFromTop(22));
    brirArea.removeFromTop(4);
    
    // Position the coordinate system explanation
    listenerCoordinateInfo.setBounds(brirArea.removeFromTop(16));
    brirArea.removeFromTop(4);
    
    // Make sure we have enough space for all listener sliders
    int remainingHeight = brirArea.getHeight();
    int requiredHeight = (brirSliderHeight + 4) * 3;
    
    if (remainingHeight < requiredHeight) {
        // Adjust global size to make room for all sliders
        setSize(getWidth(), getHeight() + (requiredHeight - remainingHeight) + 10);
    }
    
    // Position the listener position sliders with less spacing
    listenerXSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(4);
    listenerYSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    brirArea.removeFromTop(4);
    listenerZSlider.setBounds(brirArea.removeFromTop(brirSliderHeight).withTrimmedLeft(brirLabelWidth));
    
    // Position the room view component
    visualArea.removeFromTop(60); // Leave space at the top
    visualArea.removeFromBottom(60); // Leave space at the bottom
    roomView->setBounds(visualArea);
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
        
        // Update status label with current values for debugging
        if (processorRef.getOscPortConnected())
        {
            statusLabel.setText("OSC Connected on port " + juce::String(processorRef.getOscPortID()) 
                                + " - YPR: " + juce::String(processorRef.getYaw(), 1) + ", " 
                                + juce::String(processorRef.getPitch(), 1) + ", "
                                + juce::String(processorRef.getRoll(), 1),
                                juce::dontSendNotification);
        }
    }
    
    // Update BRIR status in label if enabled
    if (processorRef.getEnableBRIR())
    {
        if (!processorRef.getEnableHeadTracking() || !processorRef.getOscPortConnected())
        {
            statusLabel.setText("BRIR Enabled - Room: " + 
                               juce::String(processorRef.getRoomLength(), 1) + "m x " +
                               juce::String(processorRef.getRoomWidth(), 1) + "m x " +
                               juce::String(processorRef.getRoomHeight(), 1) + "m, Listener: (" +
                               juce::String(processorRef.getListenerX(), 2) + "m, " +
                               juce::String(processorRef.getListenerY(), 2) + "m, " +
                               juce::String(processorRef.getListenerZ(), 2) + "m)",
                               juce::dontSendNotification);
        }
    }
}

// Room View Component implementation
AudioPluginAudioProcessorEditor::RoomViewComponent::RoomViewComponent(AudioPluginAudioProcessor& p) 
    : processor(p)
{
    startTimerHz(30); // Update at 30Hz
}

void AudioPluginAudioProcessorEditor::RoomViewComponent::paint(juce::Graphics& g)
{
    // Get room dimensions
    float roomLength = processor.getRoomLength();
    float roomWidth = processor.getRoomWidth();
    float roomHeight = processor.getRoomHeight();
    
    // Get listener position
    float listenerX = processor.getListenerX();
    float listenerY = processor.getListenerY();
    float listenerZ = processor.getListenerZ();
    
    // Background
    g.fillAll(juce::Colour(20, 22, 24));
    
    // Calculate room view area
    juce::Rectangle<int> viewArea = getLocalBounds();
    
    // IMPROVEMENT: Reduce width of grid area to create gap between grid and Z slider
    int gridWidthReduction = 15; // 15px gap on the right side
    viewArea.setWidth(viewArea.getWidth() - gridWidthReduction);
    
    // Calculate scale factor (pixels per meter)
    float scaleX = (float)viewArea.getWidth() * 0.8f / roomLength;
    float scaleY = (float)viewArea.getHeight() * 0.8f / roomWidth;
    float scale = juce::jmin(scaleX, scaleY);
    
    // Calculate room center in screen coordinates
    int centerX = viewArea.getCentreX();
    int centerY = viewArea.getCentreY();
    
    // Calculate room bounds with (0,0) at center
    int roomLeft = centerX - (int)(roomLength * scale / 2);
    int roomTop = centerY - (int)(roomWidth * scale / 2);
    int roomW = (int)(roomLength * scale);
    int roomH = (int)(roomWidth * scale);
    
    // Calculate screen coordinates for listener relative to center
    int listenerViewX = centerX + (int)(listenerX * scale);
    int listenerViewY = centerY - (int)(listenerY * scale); // Flip Y for screen coordinates
    
    // Calculate screen coordinates for source using our updated sourceRoomX/Y/Z values
    int sourceViewX = centerX + (int)(sourceRoomX * scale);
    int sourceViewY = centerY - (int)(sourceRoomY * scale); // Flip Y for screen coordinates
    
    // Draw coordinate axes
    g.setColour(juce::Colours::white.withAlpha(0.5f));
    g.drawLine(centerX, roomTop, centerX, roomTop + roomH, 1.0f); // Y axis
    g.drawLine(roomLeft, centerY, roomLeft + roomW, centerY, 1.0f); // X axis
    
    // Draw room outline (top view)
    g.setColour(juce::Colours::white.withAlpha(0.5f));
    g.drawRect(roomLeft, roomTop, roomW, roomH, 1);
    
    // Draw grid lines
    g.setColour(juce::Colours::white.withAlpha(0.2f));
    // Vertical grid lines (X axis)
    for (int i = -roomLength/2 + 1; i < roomLength/2; i++) {
        int x = centerX + (int)(i * scale);
        g.drawLine(x, roomTop, x, roomTop + roomH, 1.0f);
    }
    // Horizontal grid lines (Y axis)
    for (int i = -roomWidth/2 + 1; i < roomWidth/2; i++) {
        int y = centerY + (int)(-i * scale); // Negative i because Y is flipped
        g.drawLine(roomLeft, y, roomLeft + roomW, y, 1.0f);
    }
    
    // Format the coordinates with consistent decimal places for neatness
    juce::String listenerCoord = juce::String::formatted("(%.2f,%.2f,%.2f)", listenerX, listenerY, listenerZ);
    juce::String sourceCoord = juce::String::formatted("(%.2f,%.2f,%.2f)", sourceRoomX, sourceRoomY, sourceRoomZ);
    
    // Create two separate lines for better visibility
    juce::String listenerInfo = "Listener: " + listenerCoord;
    juce::String sourceInfo = "Source: " + sourceCoord;
    
    // Set font and color for the debug info
    g.setColour(juce::Colours::white);
    g.setFont(juce::Font(13.0f, juce::Font::bold));
    
    // IMPROVEMENT: Move the coordinate display higher up to avoid blocking axis labels
    // Now draws the text 60px and 40px above the top of the room view (was 40px and 20px)
    int infoX = roomLeft - 20;
    int infoWidth = roomW + 40;
    g.drawText(listenerInfo, infoX, roomTop - 60, infoWidth, 20, juce::Justification::centred, false);
    g.drawText(sourceInfo, infoX, roomTop - 40, infoWidth, 20, juce::Justification::centred, false);
    
    // Draw listener with orientation indicator (green dot)
    g.setColour(juce::Colours::green);
    g.fillEllipse(listenerViewX - 5, listenerViewY - 5, 10, 10);
    
    // Draw orientation line based on yaw
    float yawRadians = juce::degreesToRadians(-processor.getYaw());
    float lineLength = 15.0f;
    
    int directionX = listenerViewX + static_cast<int>(lineLength * std::sin(yawRadians));
    int directionY = listenerViewY - static_cast<int>(lineLength * std::cos(yawRadians));
    
    g.setColour(juce::Colours::lightgreen);
    g.drawLine(listenerViewX, listenerViewY, directionX, directionY, 2.0f);
    
    // Draw source (orange dot)
    g.setColour(juce::Colours::orange);
    g.fillEllipse(sourceViewX - 5, sourceViewY - 5, 10, 10);
    
    // IMPROVEMENT: Adjust Z axis position to create more space between grid and Z slider
    int zBarX = viewArea.getRight() + gridWidthReduction - 30; // Moved right by gridWidthReduction
    int zBarHeight = viewArea.getHeight() - 50;
    int zBarY = 25;
    int zBarWidth = 15;
    
    g.setColour(juce::Colours::white.withAlpha(0.3f));
    g.fillRect(zBarX, zBarY, zBarWidth, zBarHeight);
    
    // Draw zero line on Z axis
    int zeroLine = zBarY + zBarHeight/2;
    g.setColour(juce::Colours::white);
    g.drawLine(zBarX - 5, zeroLine, zBarX + zBarWidth + 5, zeroLine, 1.0f);
    
    // Draw listener Z position (scale from -roomHeight/2 to +roomHeight/2)
    float zPercentage = (listenerZ + roomHeight/2) / roomHeight;
    g.setColour(juce::Colours::green);
    int listenerZPos = zBarY + (int)((1.0f - zPercentage) * zBarHeight);
    g.fillRect(zBarX, listenerZPos - 2, zBarWidth, 4);
    
    // Draw source Z position
    float sourceZPercentage = (sourceRoomZ + roomHeight/2) / roomHeight;
    g.setColour(juce::Colours::orange);
    int sourceZPos = zBarY + (int)((1.0f - sourceZPercentage) * zBarHeight);
    g.fillRect(zBarX, sourceZPos - 2, zBarWidth, 4);
    
    // Draw legends
    g.setFont(12.0f);
    g.setColour(juce::Colours::green);
    g.fillEllipse(roomLeft + 10, roomTop + 10, 8, 8);
    g.setColour(juce::Colours::white);
    g.drawText("Listener", roomLeft + 25, roomTop + 10, 60, 10, juce::Justification::centredLeft, false);
    
    g.setColour(juce::Colours::orange);
    g.fillEllipse(roomLeft + 10, roomTop + 25, 8, 8);
    g.setColour(juce::Colours::white);
    g.drawText("Source", roomLeft + 25, roomTop + 25, 60, 10, juce::Justification::centredLeft, false);
    
    // Draw coordinate labels
    g.setColour(juce::Colours::white.withAlpha(0.8f));
    g.setFont(10.0f);
    
    // X axis labels
    g.drawText("-" + juce::String(roomLength/2, 1) + "m", roomLeft - 20, centerY, 40, 15, juce::Justification::centredRight, false);
    g.drawText("+" + juce::String(roomLength/2, 1) + "m", roomLeft + roomW - 20, centerY, 40, 15, juce::Justification::centredLeft, false);
    g.drawText("X", roomLeft + roomW + 5, centerY, 15, 15, juce::Justification::centredLeft, false);
    
    // Y axis labels
    g.drawText("-" + juce::String(roomWidth/2, 1) + "m", centerX - 20, roomTop + roomH, 40, 15, juce::Justification::centredRight, false);
    g.drawText("+" + juce::String(roomWidth/2, 1) + "m", centerX - 20, roomTop - 5, 40, 15, juce::Justification::centredRight, false);
    g.drawText("Y", centerX, roomTop - 15, 15, 15, juce::Justification::centredLeft, false);
    
    // Z axis labels
    g.drawText("Z", zBarX, zBarY - 15, zBarWidth, 15, juce::Justification::centred, false);
    g.drawText("-" + juce::String(roomHeight/2, 1) + "m", zBarX - 15, zBarY + zBarHeight - 5, 15, 15, juce::Justification::centredRight, false);
    g.drawText("+" + juce::String(roomHeight/2, 1) + "m", zBarX - 15, zBarY - 5, 15, 15, juce::Justification::centredRight, false);
    g.drawText("0", zBarX - 15, zeroLine - 7, 15, 15, juce::Justification::centredRight, false);
}

void AudioPluginAudioProcessorEditor::RoomViewComponent::timerCallback()
{
    // Get room dimensions
    float roomLength = processor.getRoomLength();
    float roomWidth = processor.getRoomWidth();
    float roomHeight = processor.getRoomHeight();
    
    // Get listener position
    float listenerX = processor.getListenerX();
    float listenerY = processor.getListenerY();
    float listenerZ = processor.getListenerZ();
    
    // NEW: Directly read HRTF parameters from the processor's AudioProcessorValueTreeState
    float azimuth = *processor.apvts.getRawParameterValue("hrtf_azimuth");
    float elevation = *processor.apvts.getRawParameterValue("hrtf_elevation");
    float distance = *processor.apvts.getRawParameterValue("hrtf_distance");
    
    // Check if HRTF parameters have changed
    bool hrtfParamsChanged = !sourceInitialized || 
                           std::abs(azimuth - lastAzimuth) > 0.1f || 
                           std::abs(elevation - lastElevation) > 0.1f || 
                           std::abs(distance - lastDistance) > 0.01f;
    
    // Only update the source position if HRTF parameters changed
    if (hrtfParamsChanged) {
        // Convert spherical to Cartesian - proper spherical coordinate conversion
        float azimuthRad = juce::degreesToRadians(azimuth);
        float elevationRad = juce::degreesToRadians(elevation);
        
        // Proper spherical to Cartesian conversion
        // For azimuth 0°, source should be in front (positive Y)
        // X = distance * sin(azimuth) * cos(elevation)
        // Y = distance * cos(azimuth) * cos(elevation)
        // Z = distance * sin(elevation)
        float relX = distance * std::sin(azimuthRad) * std::cos(elevationRad);
        float relY = distance * std::cos(azimuthRad) * std::cos(elevationRad);
        float relZ = distance * std::sin(elevationRad);
        
        // FIXED: Position source relative to world origin (0,0,0) instead of relative to listener
        sourceRoomX = relX;
        sourceRoomY = relY;
        sourceRoomZ = relZ;
        
        // Clamp to room boundaries
        float halfLength = roomLength / 2.0f;
        float halfWidth = roomWidth / 2.0f;
        float halfHeight = roomHeight / 2.0f;
        
        sourceRoomX = juce::jlimit(-halfLength + 0.1f, halfLength - 0.1f, sourceRoomX);
        sourceRoomY = juce::jlimit(-halfWidth + 0.1f, halfWidth - 0.1f, sourceRoomY);
        sourceRoomZ = juce::jlimit(-halfHeight + 0.1f, halfHeight - 0.1f, sourceRoomZ);
        
        // Update tracking variables
        lastAzimuth = azimuth;
        lastElevation = elevation;
        lastDistance = distance;
        lastListenerX = listenerX;
        lastListenerY = listenerY;
        lastListenerZ = listenerZ;
        
        sourceInitialized = true;
    }
    
    repaint();
}