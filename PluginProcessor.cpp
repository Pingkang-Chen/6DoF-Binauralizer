#include "PluginProcessor.h"
#include "PluginEditor.h"
#include <cmath>

juce::AudioProcessorValueTreeState::ParameterLayout parameters() {
    std::vector<std::unique_ptr<juce::RangedAudioParameter>> parameter_list;

    // HRTF position control using spherical coordinates
    parameter_list.push_back(std::make_unique<juce::AudioParameterFloat>(
        ParameterID {"hrtf_azimuth", 1},
        "Azimuth",
        juce::NormalisableRange<float>(0.0f, 360.0f, 1.0f),
        0.0f, // Default: 0 degrees (front direction)
        "°"));

    parameter_list.push_back(std::make_unique<juce::AudioParameterFloat>(
        ParameterID {"hrtf_elevation", 1},
        "Elevation",
        juce::NormalisableRange<float>(-90.0f, 90.0f, 1.0f),
        0.0f, // Default: 0 degrees (ear level)
        "°"));

    parameter_list.push_back(std::make_unique<juce::AudioParameterFloat>(
        ParameterID {"hrtf_distance", 1},
        "Distance",
        juce::NormalisableRange<float>(0.2f, 10.0f, 0.1f, 0.5f), // Logarithmic scale for distance
        1.0f, // Default: 1 meter
        "m"));

    return { parameter_list.begin(), parameter_list.end() };
}

//==============================================================================
AudioPluginAudioProcessor::AudioPluginAudioProcessor()
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       ),
     apvts(*this, nullptr, "Parameters", parameters())
{
    // Initialize Cartesian coordinates from spherical defaults
    float values[3] = {currentAzimuth, currentElevation, currentDistance};
    mysofa_s2c(values);
    currentX = values[0];
    currentY = values[1];
    currentZ = values[2];
    
    // Initialize OSC
    osc_connected = osc.connect(osc_port_ID);
    osc.addListener(this);
}

AudioPluginAudioProcessor::~AudioPluginAudioProcessor()
{
    // Disconnect OSC
    osc.disconnect();
    osc.removeListener(this);
    
    // Clean up HRTF resources
    if (hrtf) {
        mysofa_close(hrtf);
        hrtf = nullptr;
    }
}

void AudioPluginAudioProcessor::oscMessageReceived(const juce::OSCMessage& message)
{
    // Only process OSC messages if head tracking is enabled
    if (!enableHeadTracking)
        return;
        
    // Handle yaw-pitch-roll array
    if (message.size() == 3 && message.getAddressPattern().toString().compare("/ypr")==0) {
        if (message[0].isFloat32()) {
            float yaw = message[0].getFloat32();
            if (flipYaw) yaw = -yaw;
            setYaw(yaw);
        }
        if (message[1].isFloat32()) {
            float pitch = message[1].getFloat32();
            if (flipPitch) pitch = -pitch;
            setPitch(pitch);
        }
        if (message[2].isFloat32()) {
            float roll = message[2].getFloat32();
            if (flipRoll) roll = -roll;
            setRoll(roll);
        }
        return;
    }
    
    // Handle individual rotation components
    if(message.getAddressPattern().toString().compare("/yaw")==0) {
        if (message[0].isFloat32()) {
            float yaw = message[0].getFloat32();
            if (flipYaw) yaw = -yaw;
            setYaw(yaw);
        }
    }
    else if(message.getAddressPattern().toString().compare("/pitch")==0) {
        if (message[0].isFloat32()) {
            float pitch = message[0].getFloat32();
            if (flipPitch) pitch = -pitch;
            setPitch(pitch);
        }
    }
    else if(message.getAddressPattern().toString().compare("/roll")==0) {
        if (message[0].isFloat32()) {
            float roll = message[0].getFloat32();
            if (flipRoll) roll = -roll;
            setRoll(roll);
        }
    }
}

void AudioPluginAudioProcessor::setOscPortID(int newID)
{
    osc.disconnect();
    osc_port_ID = newID;
    osc_connected = osc.connect(osc_port_ID);
}

// Apply head rotation to HRTF positioning
void AudioPluginAudioProcessor::applyHeadRotation()
{
    if (!enableHeadTracking)
        return;
        
    // Get current positions from UI sliders
    float azimuth = apvts.getParameter("hrtf_azimuth")->getValue() * 360.0f;
    float elevation = apvts.getParameter("hrtf_elevation")->getValue() * 180.0f - 90.0f;
    float distance = apvts.getParameter("hrtf_distance")->getValue() * 9.8f + 0.2f;
    
    // For direct compatibility with your working implementation:
    // Simple adjustment of azimuth and elevation
    float adjustedAzimuth = azimuth + currentYaw;
    float adjustedElevation = elevation + currentPitch;
    
    // If there's no roll, use the simple approach
    if (std::abs(currentRoll) < 0.01f) {
        updateHRTFPositionSpherical(adjustedAzimuth, adjustedElevation, distance);
        return;
    }
    
    // For roll, we need the full 3D transformation
    // First convert adjusted spherical to Cartesian
    float x, y, z;
    sphericalToCartesian(adjustedAzimuth, adjustedElevation, distance, x, y, z);
    
    // Apply roll rotation matrix (around Z axis)
    float rollRad = juce::degreesToRadians(currentRoll);
    float cosRoll = std::cos(rollRad);
    float sinRoll = std::sin(rollRad);
    
    float newX = x * cosRoll - y * sinRoll;
    float newY = x * sinRoll + y * cosRoll;
    float newZ = z;
    
    // Convert back to spherical
    float finalAzimuth, finalElevation, finalDistance;
    cartesianToSpherical(newX, newY, newZ, finalAzimuth, finalElevation, finalDistance);
    
    // Update HRTF with final coordinates
    updateHRTFPositionSpherical(finalAzimuth, finalElevation, distance);
}

//==============================================================================
const juce::String AudioPluginAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool AudioPluginAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool AudioPluginAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool AudioPluginAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double AudioPluginAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int AudioPluginAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int AudioPluginAudioProcessor::getCurrentProgram()
{
    return 0;
}

void AudioPluginAudioProcessor::setCurrentProgram (int index)
{
    juce::ignoreUnused (index);
}

const juce::String AudioPluginAudioProcessor::getProgramName (int index)
{
    juce::ignoreUnused (index);
    return {};
}

void AudioPluginAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
    juce::ignoreUnused (index, newName);
}

//==============================================================================
void AudioPluginAudioProcessor::prepareToPlay(double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback initialization that you need.
    
    // Reset convolution processors
    convolutionL.reset();
    convolutionR.reset();
    
    juce::File sofaFile = juce::File::getSpecialLocation(juce::File::userDesktopDirectory).getChildFile("SCUT_NF_subject0006_measured.sofa");
    if (!sofaFile.exists()) {
        std::cout << "SOFA file does not exist: " << sofaFile.getFullPathName() << std::endl;
        exit(1);
    }
    
    if (!loadHRTF(sofaFile)) {
        std::cout << "Failed to load HRTF file: " << sofaFile.getFullPathName() << std::endl;
        exit(1);
    }

    // Initialize parameter smoothing with fast response for testing
    azimuthSmoothed.reset(sampleRate, 0.02f);   // 20ms smoothing time
    elevationSmoothed.reset(sampleRate, 0.02f);
    distanceSmoothed.reset(sampleRate, 0.02f);
    
    // Initialize HRTF position from parameters (using spherical coordinates)
    float azimuth = apvts.getParameter("hrtf_azimuth")->getValue() * 360.0f;
    float elevation = apvts.getParameter("hrtf_elevation")->getValue() * 180.0f - 90.0f;
    float distance = apvts.getParameter("hrtf_distance")->getValue() * 9.8f + 0.2f;
    
    // Set initial values for smoothed parameters
    azimuthSmoothed.setCurrentAndTargetValue(azimuth);
    elevationSmoothed.setCurrentAndTargetValue(elevation);
    distanceSmoothed.setCurrentAndTargetValue(distance);
    
    // Apply initial HRTF position
    updateHRTFPositionSpherical(azimuth, elevation, distance);
    
    // Prepare the convolution processors
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = static_cast<uint32>(samplesPerBlock);
    spec.numChannels = 1; // Each convolution processor handles one channel
    
    convolutionL.prepare(spec);
    convolutionR.prepare(spec);
    
    // Prepare the air absorption filters
    juce::dsp::ProcessSpec airSpec;
    airSpec.sampleRate = sampleRate;
    airSpec.maximumBlockSize = static_cast<uint32>(samplesPerBlock);
    airSpec.numChannels = 2; // Stereo processing for the final HRTF buffer
    
    highShelfFilter.prepare(airSpec);
    highPassFilter.prepare(airSpec);
    
    // Set default coefficients to pass-through
    *highShelfFilter.state = *juce::dsp::IIR::Coefficients<float>::makeHighShelf(
        getSampleRate(), 5000.0f, 1.0f, 1.0f);
    
    *highPassFilter.state = *juce::dsp::IIR::Coefficients<float>::makeHighPass(
        getSampleRate(), 20.0f, 0.7f);
}


void AudioPluginAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

bool AudioPluginAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}

void AudioPluginAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer,
    juce::MidiBuffer& midiMessages)
{
    juce::ignoreUnused(midiMessages);

    juce::ScopedNoDenormals noDenormals; 
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();

    // In case we have more outputs than inputs, clear any output
    // channels that didn't contain input data
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());

    // Create a copy of the input buffer to preserve the original input
    juce::AudioBuffer<float> inputCopy;
    inputCopy.makeCopyOf(buffer);

    // Clear the output buffer to ensure we're starting with a clean slate
    buffer.clear();

    if (enableHeadTracking) {
        // When head tracking is enabled, call our method to apply the rotation
        applyHeadRotation();
    } else {
        // Original code for position updates from UI sliders
        // Get raw parameter values for HRTF position from UI
        float rawAzimuth = apvts.getParameter("hrtf_azimuth")->getValue();
        float rawElevation = apvts.getParameter("hrtf_elevation")->getValue();
        float rawDistance = apvts.getParameter("hrtf_distance")->getValue();
        
        // Convert normalized parameter values to real-world values
        float azimuthTarget = rawAzimuth * 360.0f;
        float elevationTarget = rawElevation * 180.0f - 90.0f;
        float distanceTarget = rawDistance * 9.8f + 0.2f;

        // Update smoothed values with new targets
        azimuthSmoothed.setTargetValue(azimuthTarget);
        elevationSmoothed.setTargetValue(elevationTarget);
        distanceSmoothed.setTargetValue(distanceTarget);

        // Process each sample to get properly smoothed values
        // Use vectors instead of variable-length arrays
        std::vector<float> azimuthValues(buffer.getNumSamples());
        std::vector<float> elevationValues(buffer.getNumSamples());
        std::vector<float> distanceValues(buffer.getNumSamples());

        for (int i = 0; i < buffer.getNumSamples(); ++i) {
            azimuthValues[i] = azimuthSmoothed.getNextValue();
            elevationValues[i] = elevationSmoothed.getNextValue();
            distanceValues[i] = distanceSmoothed.getNextValue();
        }

        // Use the last value in each array as the current value
        float azimuth = azimuthValues[buffer.getNumSamples() - 1];
        float elevation = elevationValues[buffer.getNumSamples() - 1];
        float distance = distanceValues[buffer.getNumSamples() - 1];

        // Check if we should update the HRTF (only if parameters changed significantly and enough time has passed)
        float currentTime = static_cast<float>(juce::Time::getMillisecondCounterHiRes() / 1000.0);
        bool significantChange = std::abs(azimuth - currentAzimuth) > 0.5f || 
                                std::abs(elevation - currentElevation) > 0.5f || 
                                std::abs(distance - currentDistance) > 0.02f;

        if (significantChange && (currentTime - lastHRTFUpdateTime) > minTimeBetweenHRTFUpdates) {
            // Use a ScopedLock to ensure thread safety when updating convolution
            const juce::ScopedLock sl(processLock);

            std::cout << "Updating HRTF position: azimuth=" << azimuth 
                      << "°, elevation=" << elevation 
                      << "°, distance=" << distance << "m" << std::endl;

            updateHRTFPositionSpherical(azimuth, elevation, distance);
            lastHRTFUpdateTime = currentTime;
        }
    }

    // Create temporary buffers for the left and right convolution results
    juce::AudioBuffer<float> leftConvBuffer;
    juce::AudioBuffer<float> rightConvBuffer;
    leftConvBuffer.setSize(1, buffer.getNumSamples());
    rightConvBuffer.setSize(1, buffer.getNumSamples());

    // If we have a mono input, use it directly
    if (totalNumInputChannels == 1) {
        // Copy the mono input to both temporary buffers
        leftConvBuffer.copyFrom(0, 0, inputCopy, 0, 0, buffer.getNumSamples());
        rightConvBuffer.copyFrom(0, 0, inputCopy, 0, 0, buffer.getNumSamples());
    }
    // For any multi-channel input (stereo or more), downmix to mono
    else if (totalNumInputChannels >= 2) {
        leftConvBuffer.clear();
        rightConvBuffer.clear();
        
        // Mix down ALL channels to mono
        for (int i = 0; i < buffer.getNumSamples(); ++i) {
            float monoSample = 0.0f;
            for (int channel = 0; channel < totalNumInputChannels; ++channel) {
                monoSample += inputCopy.getSample(channel, i);
            }
            monoSample /= static_cast<float>(totalNumInputChannels); // Average all channels
            
            leftConvBuffer.setSample(0, i, monoSample);
            rightConvBuffer.setSample(0, i, monoSample);
        }
    }

    // Apply convolution to left channel (using left IR)
    juce::dsp::AudioBlock<float> leftBlock(leftConvBuffer);
    convolutionL.process(juce::dsp::ProcessContextReplacing<float>(leftBlock));

    // Apply convolution to right channel (using right IR)
    juce::dsp::AudioBlock<float> rightBlock(rightConvBuffer);
    convolutionR.process(juce::dsp::ProcessContextReplacing<float>(rightBlock));

    // Create a buffer for HRTF processed signal
    juce::AudioBuffer<float> hrtfBuffer;
    hrtfBuffer.setSize(2, buffer.getNumSamples());

    // Copy the processed signals to the HRTF buffer
    hrtfBuffer.copyFrom(0, 0, leftConvBuffer, 0, 0, buffer.getNumSamples());
    hrtfBuffer.copyFrom(1, 0, rightConvBuffer, 0, 0, buffer.getNumSamples());

    // For near-field distances (within measurement range), use natural HRTF amplitude
    // For distances beyond measurement range, apply inverse square law and air absorption
    float maxMeasuredDistance = 1.0f; // Maximum distance in your SOFA measurements
    float distanceAttenuation = 1.0f;

    if (currentDistance > maxMeasuredDistance) {
        // Apply inverse square law with reference to the max measured distance
        float referenceGain = 1.0f / (maxMeasuredDistance * maxMeasuredDistance);
        float currentGain = 1.0f / (currentDistance * currentDistance);
        distanceAttenuation = currentGain / referenceGain;
        
        // Apply the attenuation to the HRTF buffer
        hrtfBuffer.applyGain(distanceAttenuation);
        
        // Calculate excess distance (beyond measured range)
        float excessDistance = currentDistance - maxMeasuredDistance;
        
        // Set up filter bank for physically-based air absorption
        
        // 1. High-frequency shelf filter (affects 4kHz+)
        float highShelfFreq = 4000.0f;
        float highShelfGain = std::pow(10.0f, -0.2f * excessDistance / 20.0f);
        float highShelfQ = 0.7f;
        
        // 2. High-pass filter to simulate very low frequency boost
        float highPassFreq = 20.0f + excessDistance * 5.0f;
        float highPassQ = 0.7f;
        
        // Update filter coefficients
        *highShelfFilter.state = *juce::dsp::IIR::Coefficients<float>::makeHighShelf(
            getSampleRate(), highShelfFreq, highShelfQ, highShelfGain);
            
        *highPassFilter.state = *juce::dsp::IIR::Coefficients<float>::makeHighPass(
            getSampleRate(), highPassFreq, highPassQ);
        
        // Apply filters
        juce::dsp::AudioBlock<float> block(hrtfBuffer);
        juce::dsp::ProcessContextReplacing<float> context(block);
        
        highShelfFilter.process(context);
        highPassFilter.process(context);
    }
    else {
        // Just apply the natural amplitude for distances <= 1m
        // No additional attenuation needed as the non-normalized HRTFs already have proper amplitudes
    }

    // Copy the fully processed HRTF signal to the output buffer
    buffer.copyFrom(0, 0, hrtfBuffer, 0, 0, buffer.getNumSamples());
    buffer.copyFrom(1, 0, hrtfBuffer, 1, 0, buffer.getNumSamples());

    // Apply a fixed compensation gain to maintain consistent volume
    // This value can be adjusted based on testing with different source material
    float fixedCompensationGain = 4.0f; // 12dB boost (4.0 = 10^(12/20))
    buffer.applyGain(fixedCompensationGain);
}

//==============================================================================
bool AudioPluginAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* AudioPluginAudioProcessor::createEditor()
{
    return new AudioPluginAudioProcessorEditor (*this);
}

//==============================================================================
void AudioPluginAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // Store parameters in memory block using XML
    auto state = apvts.copyState();
    std::unique_ptr<juce::XmlElement> xml(state.createXml());
    
    // Store head tracking parameters
    xml->setAttribute("enableHeadTracking", enableHeadTracking);
    xml->setAttribute("yaw", static_cast<double>(currentYaw));
    xml->setAttribute("pitch", static_cast<double>(currentPitch));
    xml->setAttribute("roll", static_cast<double>(currentRoll));
    xml->setAttribute("flipYaw", flipYaw);
    xml->setAttribute("flipPitch", flipPitch);
    xml->setAttribute("flipRoll", flipRoll);
    xml->setAttribute("useRollPitchYaw", useRollPitchYaw);
    xml->setAttribute("oscPort", osc_port_ID);
    
    copyXmlToBinary(*xml, destData);
}

void AudioPluginAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // Restore parameters from memory block
    std::unique_ptr<juce::XmlElement> xmlState(getXmlFromBinary(data, sizeInBytes));
    
    if (xmlState.get() != nullptr) {
        if (xmlState->hasTagName(apvts.state.getType())) {
            apvts.replaceState(juce::ValueTree::fromXml(*xmlState));
            
            // Restore head tracking parameters
            enableHeadTracking = xmlState->getBoolAttribute("enableHeadTracking", false);
            currentYaw = static_cast<float>(xmlState->getDoubleAttribute("yaw", 0.0));
            currentPitch = static_cast<float>(xmlState->getDoubleAttribute("pitch", 0.0));
            currentRoll = static_cast<float>(xmlState->getDoubleAttribute("roll", 0.0));
            flipYaw = xmlState->getBoolAttribute("flipYaw", false);
            flipPitch = xmlState->getBoolAttribute("flipPitch", false);
            flipRoll = xmlState->getBoolAttribute("flipRoll", false);
            useRollPitchYaw = xmlState->getBoolAttribute("useRollPitchYaw", false);
            
            // Restore OSC port and reconnect
            int savedPort = xmlState->getIntAttribute("oscPort", 9000);
            if (savedPort != osc_port_ID) {
                setOscPortID(savedPort);
            }
        }
    }
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new AudioPluginAudioProcessor();
}

//==============================================================================
// HRTF methods
//==============================================================================

bool AudioPluginAudioProcessor::loadHRTF(const juce::File& sofaFile) 
{
    // Clean up any existing HRTF data
    if (hrtf) {
        mysofa_close(hrtf);
        hrtf = nullptr;
    }
    
    int err;
    // Use non-normalized loading to preserve original amplitude variations
    hrtf = mysofa_open_no_norm(sofaFile.getFullPathName().toRawUTF8(), getSampleRate(), &hrtfFilterLength, &err);
    
    if (!hrtf) {
        std::cout << "Error loading SOFA file: " << err << std::endl;
        return false;
    }
    
    // Resize our IR buffers
    leftIR.resize(hrtfFilterLength);
    rightIR.resize(hrtfFilterLength);
    
    // Print some debug info
    std::cout << "Successfully loaded HRTF file with filter length: " << hrtfFilterLength << std::endl;
    
    // Update HRTF for initial position (this will fill the IR buffers)
    updateHRTFPositionSpherical(currentAzimuth, currentElevation, currentDistance);
    
    return true;
}

bool AudioPluginAudioProcessor::updateHRTFPositionSpherical(float azimuth, float elevation, float distance)
{
    std::cout << "ENTERED updateHRTFPositionSpherical with values: azimuth=" 
              << azimuth << "°, elevation=" << elevation 
              << "°, distance=" << distance << "m" << std::endl;
              
    if (!hrtf) {
        std::cout << "ERROR: HRTF is null!" << std::endl;
        return false;
    }
    
    // Invert the azimuth to correct left-right reversal (clockwise to counterclockwise)
    azimuth = 360.0f - azimuth;
    
    // Ensure azimuth is in 0-360 range
    azimuth = fmod(azimuth + 360.0f, 360.0f);
    
    // Update current spherical coordinates
    currentAzimuth = azimuth;
    currentElevation = elevation;
    currentDistance = distance;
    
    // Convert spherical to Cartesian coordinates using libmysofa
    float values[3] = {azimuth, elevation, distance};
    mysofa_s2c(values);
    
    // Update current Cartesian coordinates
    currentX = values[0];
    currentY = values[1];
    currentZ = values[2];
    
    std::cout << "Cartesian position: x=" << currentX 
              << ", y=" << currentY 
              << ", z=" << currentZ << std::endl;
    
    // Get HRTF filters for this position
    mysofa_getfilter_float(hrtf, currentX, currentY, currentZ, leftIR.data(), rightIR.data(), &leftDelay, &rightDelay);
    
    // Create AudioBuffers from the IR data
    juce::AudioBuffer<float> leftBuffer(1, hrtfFilterLength);
    juce::AudioBuffer<float> rightBuffer(1, hrtfFilterLength);
    
    // Copy IR data to the AudioBuffers
    for (int i = 0; i < hrtfFilterLength; ++i) {
        leftBuffer.setSample(0, i, leftIR[i]);
        rightBuffer.setSample(0, i, rightIR[i]);
    }
    
    // Load the new impulse responses with normalization disabled to preserve amplitude differences
    convolutionL.loadImpulseResponse(std::move(leftBuffer), 
                                    getSampleRate(),
                                    juce::dsp::Convolution::Stereo::no, 
                                    juce::dsp::Convolution::Trim::no, 
                                    juce::dsp::Convolution::Normalise::no);
    
    convolutionR.loadImpulseResponse(std::move(rightBuffer), 
                                    getSampleRate(),
                                    juce::dsp::Convolution::Stereo::no, 
                                    juce::dsp::Convolution::Trim::no, 
                                    juce::dsp::Convolution::Normalise::no);
    
    std::cout << "Updated HRTF filters. Left delay: " << leftDelay << ", Right delay: " << rightDelay << std::endl;
    
    return true;
}

bool AudioPluginAudioProcessor::updateHRTFPosition(float x, float y, float z)
{
    if (!hrtf) {
        return false;
    }
    
    // Update current Cartesian coordinates
    currentX = x;
    currentY = y;
    currentZ = z;
    
    // Convert to spherical coordinates for UI display and storage
    cartesianToSpherical(x, y, z, currentAzimuth, currentElevation, currentDistance);
    
    // Get HRTF filters for this position
    mysofa_getfilter_float(hrtf, x, y, z, leftIR.data(), rightIR.data(), &leftDelay, &rightDelay);
    
    // Create AudioBuffers from the IR data
    juce::AudioBuffer<float> leftBuffer(1, hrtfFilterLength);
    juce::AudioBuffer<float> rightBuffer(1, hrtfFilterLength);
    
    // Copy IR data to the AudioBuffers
    for (int i = 0; i < hrtfFilterLength; ++i) {
        leftBuffer.setSample(0, i, leftIR[i]);
        rightBuffer.setSample(0, i, rightIR[i]);
    }
    
    // Load the new impulse responses with normalization disabled to preserve amplitude differences
    convolutionL.loadImpulseResponse(std::move(leftBuffer), 
                                    getSampleRate(),
                                    juce::dsp::Convolution::Stereo::no, 
                                    juce::dsp::Convolution::Trim::no, 
                                    juce::dsp::Convolution::Normalise::no);
    
    convolutionR.loadImpulseResponse(std::move(rightBuffer), 
                                    getSampleRate(),
                                    juce::dsp::Convolution::Stereo::no, 
                                    juce::dsp::Convolution::Trim::no, 
                                    juce::dsp::Convolution::Normalise::no);
    
    return true;
}

// Helper method to convert Cartesian to spherical coordinates (not using libmysofa)
void AudioPluginAudioProcessor::cartesianToSpherical(float x, float y, float z, float& azimuth, float& elevation, float& distance)
{
    float values[3] = {x, y, z};
    mysofa_c2s(values);
    azimuth = values[0];
    elevation = values[1];
    distance = values[2];
}

// Helper method to convert spherical to Cartesian coordinates (not using libmysofa)
void AudioPluginAudioProcessor::sphericalToCartesian(float azimuth, float elevation, float distance, float& x, float& y, float& z)
{
    float values[3] = {azimuth, elevation, distance};
    mysofa_s2c(values);
    x = values[0];
    y = values[1];
    z = values[2];
}