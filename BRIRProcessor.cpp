#include "BRIRProcessor.h"
#include "PluginProcessor.h"

BRIRProcessor::BRIRProcessor(AudioPluginAudioProcessor& processor)
    : processorRef(processor)
{
    // Initialize IR buffers
    directPathIR.setSize(2, 512);  // Stereo HRIR
    earlyReflectionsIR.setSize(2, 2048); // Early reflections (larger for room size)
    combinedBRIR.setSize(2, 4096);  // Combined BRIR (even larger to accommodate reverb tail)
    lateReverbBuffer.setSize(2, 8192); // Late reverb buffer
    tempBuffer.setSize(2, 8192);     // Temp working buffer
}

BRIRProcessor::~BRIRProcessor()
{
    // Clean up resources
}

void BRIRProcessor::prepare(double sr, int samplesPerBlock)
{
    sampleRate = sr;
    
    // Initialize convolution engines
    juce::dsp::ProcessSpec spec;
    spec.sampleRate = sampleRate;
    spec.maximumBlockSize = static_cast<uint32>(samplesPerBlock);
    spec.numChannels = 1;  // Each convolution processor handles one channel
    
    directConvolutionL.prepare(spec);
    directConvolutionR.prepare(spec);
    earlyConvolutionL.prepare(spec);
    earlyConvolutionR.prepare(spec);
    
    // Create Schroeder reverberator
    schroederReverb = std::make_unique<SchroederReverb>(reverbTime, sampleRate);
    
    // Update BRIR with current settings
    updateBRIR();
}

// Helper methods for coordinate conversion
std::array<float, 3> BRIRProcessor::centerToCornerCoordinates(const std::array<float, 3>& centerCoords) const
{
    return {
        centerCoords[0] + roomLength / 2.0f,
        centerCoords[1] + roomWidth / 2.0f,
        centerCoords[2] + roomHeight / 2.0f
    };
}

std::array<float, 3> BRIRProcessor::cornerToCenterCoordinates(const std::array<float, 3>& cornerCoords) const
{
    return {
        cornerCoords[0] - roomLength / 2.0f,
        cornerCoords[1] - roomWidth / 2.0f,
        cornerCoords[2] - roomHeight / 2.0f
    };
}

void BRIRProcessor::setRoomDimensions(float length, float width, float height)
{
    roomLength = length;
    roomWidth = width;
    roomHeight = height;
    
    // Update BRIR when room size changes
    if (isEnabled)
        updateBRIR();
}

void BRIRProcessor::setReverbTime(float t60)
{
    reverbTime = t60;
    
    // Recreate Schroeder reverberator with new RT60
    schroederReverb = std::make_unique<SchroederReverb>(reverbTime, sampleRate);
    
    // Update BRIR when reverb time changes
    if (isEnabled)
        updateBRIR();
}

void BRIRProcessor::setSourcePosition(float x, float y, float z)
{
    // Convert from centered coordinates to corner coordinates for internal use
    std::array<float, 3> centeredPosition = {x, y, z};
    std::array<float, 3> cornerPosition = centerToCornerCoordinates(centeredPosition);
    
    sourcePosition = cornerPosition;
    
    // Update BRIR when source position changes
    if (isEnabled)
        updateBRIR();
}

void BRIRProcessor::setListenerPosition(float x, float y, float z)
{
    // Convert from centered coordinates to corner coordinates for internal use
    std::array<float, 3> centeredPosition = {x, y, z};
    std::array<float, 3> cornerPosition = centerToCornerCoordinates(centeredPosition);
    
    listenerPosition = cornerPosition;
    
    // Update BRIR when listener position changes
    if (isEnabled)
        updateBRIR();
}

std::array<float, 3> BRIRProcessor::getSourcePositionCentered() const
{
    return cornerToCenterCoordinates(sourcePosition);
}

std::array<float, 3> BRIRProcessor::getListenerPositionCentered() const
{
    return cornerToCenterCoordinates(listenerPosition);
}

void BRIRProcessor::setWallAbsorption(float leftWall, float rightWall, float frontWall,
                                     float backWall, float ceiling, float floor)
{
    wallAbsorption = {leftWall, rightWall, frontWall, backWall, ceiling, floor};
    
    // Update BRIR when absorption changes
    if (isEnabled)
        updateBRIR();
}

float BRIRProcessor::calculateDistance(const std::array<float, 3>& pos1, const std::array<float, 3>& pos2)
{
    float dx = pos1[0] - pos2[0];
    float dy = pos1[1] - pos2[1];
    float dz = pos1[2] - pos2[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

void BRIRProcessor::calculateImageSources()
{
    // Clear previous virtual sources
    virtualSources.clear();
    
    // Direct path (original source) - this is already handled by HRTF in the main processor
    // But we'll store it here for reference
    float directDistance = calculateDistance(sourcePosition, listenerPosition);
    virtualSources.emplace_back(sourcePosition, directDistance, 
                              directDistance * sampleRate / speedOfSound,
                              1.0f / directDistance, 0, std::array<int, 3>{0, 0, 0});
    
    // Calculate image sources up to max reflection order
    // Based on the method in the paper, using Equations 11.12-11.14 from the Image Source Method
    
    // We'll use i, j, k as reflection indices for x, y, z axes
    for (int i = -maxReflectionOrder; i <= maxReflectionOrder; ++i) {
        for (int j = -maxReflectionOrder; j <= maxReflectionOrder; ++j) {
            for (int k = -maxReflectionOrder; k <= maxReflectionOrder; ++k) {
                // Skip direct path (already handled)
                if (i == 0 && j == 0 && k == 0) continue;
                
                // Skip if total order exceeds max reflection order
                int totalOrder = std::abs(i) + std::abs(j) + std::abs(k);
                if (totalOrder > maxReflectionOrder) continue;
                
                // Calculate virtual source position using image source method equations
                // as described in the paper (see page 264, equations 11.12-11.14)
                
                // Calculate image source coordinates using the paper's formulas
                float xi, yj, zk;
                
                // X-coordinate (uses room length)
                if (i % 2 == 0) // i even
                    xi = i * roomLength + sourcePosition[0];
                else // i odd
                    xi = (i + 1) * roomLength - sourcePosition[0];
                
                // Y-coordinate (uses room width)
                if (j % 2 == 0) // j even
                    yj = j * roomWidth + sourcePosition[1];
                else // j odd
                    yj = (j + 1) * roomWidth - sourcePosition[1];
                
                // Z-coordinate (uses room height)
                if (k % 2 == 0) // k even
                    zk = k * roomHeight + sourcePosition[2];
                else // k odd
                    zk = (k + 1) * roomHeight - sourcePosition[2];
                
                // Virtual source position
                std::array<float, 3> virtualPos = {xi, yj, zk};
                
                // Calculate distance from virtual source to listener
                float dist = calculateDistance(virtualPos, listenerPosition);
                
                // Calculate delay in samples
                float delaySamples = static_cast<float>(dist * sampleRate / speedOfSound);
                
                // Calculate amplitude (inverse distance attenuation)
                float amplitude = 1.0f / dist;
                
                // Calculate reflection coefficient
                float reflectionCoeff = calculateReflectionCoefficient(i, j, k);
                
                // Adjust amplitude by reflection coefficient
                amplitude *= reflectionCoeff;
                
                // Store virtual source
                virtualSources.emplace_back(
                    virtualPos, dist, delaySamples, amplitude, 
                    totalOrder, std::array<int, 3>{i, j, k}
                );
            }
        }
    }
    
    // Sort virtual sources by delay (increasing order)
    std::sort(virtualSources.begin(), virtualSources.end(), 
              [](const VirtualSource& a, const VirtualSource& b) {
                  return a.delay < b.delay;
              });
}

// Helper method to calculate reflection coefficient
float BRIRProcessor::calculateReflectionCoefficient(int i, int j, int k)
{
    float reflectionCoeff = 1.0f;
    
    // X-axis walls (reflection coefficient = 1 - absorption)
    int xReflections = std::abs(i);
    if (xReflections > 0) {
        float leftCoeff = 1.0f - wallAbsorption[0];  // Left wall
        float rightCoeff = 1.0f - wallAbsorption[1]; // Right wall
        
        // Apply reflection coefficients based on odd/even formula as in eq. 11.19
        if (i % 2 == 0) { // i even
            reflectionCoeff *= static_cast<float>(std::pow(leftCoeff * rightCoeff, xReflections / 2));
        } else { // i odd
            reflectionCoeff *= static_cast<float>(std::pow(leftCoeff, (xReflections+1)/2) * 
                              std::pow(rightCoeff, (xReflections-1)/2));
        }
    }
    
    // Y-axis walls
    int yReflections = std::abs(j);
    if (yReflections > 0) {
        float frontCoeff = 1.0f - wallAbsorption[2];  // Front wall
        float backCoeff = 1.0f - wallAbsorption[3];   // Back wall
        
        if (j % 2 == 0) { // j even
            reflectionCoeff *= static_cast<float>(std::pow(frontCoeff * backCoeff, yReflections / 2));
        } else { // j odd
            reflectionCoeff *= static_cast<float>(std::pow(frontCoeff, (yReflections+1)/2) * 
                              std::pow(backCoeff, (yReflections-1)/2));
        }
    }
    
    // Z-axis walls
    int zReflections = std::abs(k);
    if (zReflections > 0) {
        float ceilingCoeff = 1.0f - wallAbsorption[4]; // Ceiling
        float floorCoeff = 1.0f - wallAbsorption[5];   // Floor
        
        if (k % 2 == 0) { // k even
            reflectionCoeff *= static_cast<float>(std::pow(ceilingCoeff * floorCoeff, zReflections / 2));
        } else { // k odd
            reflectionCoeff *= static_cast<float>(std::pow(ceilingCoeff, (zReflections+1)/2) * 
                              std::pow(floorCoeff, (zReflections-1)/2));
        }
    }
    
    return reflectionCoeff;
}

void BRIRProcessor::updateBRIR()
{
    // Calculate image sources for the room
    calculateImageSources();
    
    // Clear BRIR buffers
    directPathIR.clear();
    earlyReflectionsIR.clear();
    combinedBRIR.clear();
    
    // Direct path IR will be handled by HRTF convolution in the main processor
    
    // For early reflections, we'll set up the FIR filter with delays and amplitudes
    // For each virtual source (except direct path), we'll need to:
    // 1. Calculate HRTF for its direction
    // 2. Apply appropriate delay and amplitude
    
    // Calculate max delay for early reflections buffer sizing
    float maxDelay = 0.0f;
    for (const auto& vs : virtualSources) {
        if (vs.reflectionOrder > 0 && vs.reflectionOrder <= maxReflectionOrder) {
            maxDelay = std::max(maxDelay, vs.delay);
        }
    }
    
    // Ensure early reflections buffer is large enough
    int earlyReflectionsSize = static_cast<int>(maxDelay) + 512; // Add HRIR length
    earlyReflectionsIR.setSize(2, earlyReflectionsSize);
    
    // For demonstration: We'll get the HRIR for each virtual source direction
    // and place it at the appropriate delay in the earlyReflectionsIR
    
    // Process early reflections (excluding direct path)
    for (size_t i = 1; i < virtualSources.size(); ++i) {
        const auto& vs = virtualSources[i];
        
        // Skip if this isn't an early reflection
        if (vs.reflectionOrder > maxReflectionOrder) continue;
        
        // Calculate azimuth and elevation to virtual source
        // Convert Cartesian to spherical coordinates (we need direction from listener to virtual source)
        float dx = vs.position[0] - listenerPosition[0];
        float dy = vs.position[1] - listenerPosition[1];
        float dz = vs.position[2] - listenerPosition[2];
        
        // Calculate azimuth (0° is front, 90° is right, etc.)
        float azimuth = std::atan2(dx, dy) * 180.0f / juce::MathConstants<float>::pi;
        if (azimuth < 0) azimuth += 360.0f;  // Convert to 0-360 range
        
        // Calculate elevation (-90° to +90°, 0° is horizontal)
        float elevation = std::atan2(dz, std::sqrt(dx*dx + dy*dy)) * 180.0f / juce::MathConstants<float>::pi;
        
        // Use elevation for HRIR selection (but for now we'll mark it as used)
        juce::ignoreUnused(elevation);
        
        // Get HRIR for this direction (using our main processor's HRTF functionality)
        // This will be a simplified version - in practice, you'd need to implement this properly
        // to get the HRIR for the specific direction
        
        // We'll need access to the HRTF data from the main processor
        // For now, let's assume we can get HRIR data for a specific direction
        juce::AudioBuffer<float> hrir(2, 512);  // Sample HRIR size
        
        // In practice, you would use something like:
        // processorRef.getHRIRForDirection(azimuth, elevation, vs.distance, hrir);
        
        // For the sake of the example, we'll just create a simplified HRIR
        // In reality, you would get the actual HRIR data from the SOFA file
        hrir.clear();
        
        // Create a simple "placeholder" HRIR - a simple impulse with some decay
        // Left channel
        hrir.setSample(0, 0, 0.9f * vs.amplitude);
        for (int s = 1; s < 20; ++s)
            hrir.setSample(0, s, 0.9f * vs.amplitude * std::exp(-s / 5.0f));
        
        // Right channel (slight delay for interaural difference)
        int itd = (azimuth > 180.0f) ? 2 : 0;  // Simple ITD model
        hrir.setSample(1, 0 + itd, 0.9f * vs.amplitude);
        for (int s = 1; s < 20; ++s)
            hrir.setSample(1, s + itd, 0.9f * vs.amplitude * std::exp(-s / 5.0f));
        
        // Add delayed HRIR to early reflections IR
        int delayOffset = static_cast<int>(vs.delay);
        if (delayOffset + hrir.getNumSamples() <= earlyReflectionsIR.getNumSamples()) {
            for (int channel = 0; channel < 2; ++channel) {
                for (int sample = 0; sample < hrir.getNumSamples(); ++sample) {
                    float currentSample = earlyReflectionsIR.getSample(channel, delayOffset + sample);
                    earlyReflectionsIR.setSample(channel, delayOffset + sample, 
                                               currentSample + hrir.getSample(channel, sample));
                }
            }
        }
    }
    
    // Load early reflections IR into convolution engines
    juce::AudioBuffer<float> leftEarlyIR(1, earlyReflectionsIR.getNumSamples());
    juce::AudioBuffer<float> rightEarlyIR(1, earlyReflectionsIR.getNumSamples());
    
    // Copy data to mono buffers for convolution
    for (int s = 0; s < earlyReflectionsIR.getNumSamples(); ++s) {
        leftEarlyIR.setSample(0, s, earlyReflectionsIR.getSample(0, s));
        rightEarlyIR.setSample(0, s, earlyReflectionsIR.getSample(1, s));
    }
    
    // Load impulse responses into convolution engines
    earlyConvolutionL.loadImpulseResponse(std::move(leftEarlyIR),
                                        sampleRate,
                                        juce::dsp::Convolution::Stereo::no,
                                        juce::dsp::Convolution::Trim::no,
                                        juce::dsp::Convolution::Normalise::no);
    
    earlyConvolutionR.loadImpulseResponse(std::move(rightEarlyIR),
                                        sampleRate,
                                        juce::dsp::Convolution::Stereo::no,
                                        juce::dsp::Convolution::Trim::no,
                                        juce::dsp::Convolution::Normalise::no);
}

void BRIRProcessor::process(juce::AudioBuffer<float>& buffer)
{
    if (!isEnabled || buffer.getNumChannels() < 2)
        return;
    
    // Create temporary buffers for processing
    tempBuffer.setSize(2, buffer.getNumSamples(), false, false, true);
    tempBuffer.clear();
    
    // Early reflections convolution
    // Process left channel
    juce::dsp::AudioBlock<float> leftBlock(tempBuffer);
    juce::dsp::AudioBlock<float> sourceLeftBlock(buffer);
    
    // Create references to single channels
    auto leftChannelBlock = leftBlock.getSingleChannelBlock(0);
    auto sourceLeftChannelBlock = sourceLeftBlock.getSingleChannelBlock(0);
    
    earlyConvolutionL.process(juce::dsp::ProcessContextNonReplacing<float>(sourceLeftChannelBlock, leftChannelBlock));
    
    // Process right channel
    auto rightChannelBlock = leftBlock.getSingleChannelBlock(1);
    auto sourceRightChannelBlock = sourceLeftBlock.getSingleChannelBlock(1);
    
    earlyConvolutionR.process(juce::dsp::ProcessContextNonReplacing<float>(sourceRightChannelBlock, rightChannelBlock));
    
    // Set up late reverberation buffer
    lateReverbBuffer.setSize(2, buffer.getNumSamples(), false, false, true);
    lateReverbBuffer.clear();
    
    // Copy buffer to late reverb (this will be processed with Schroeder reverb)
    // We apply a delay based on late reverb onset time
    int lateReverbDelaySamples = static_cast<int>(lateReverbOnset * sampleRate / 1000.0f);
    
    // Copy with delay (if there's enough space)
    if (lateReverbDelaySamples < buffer.getNumSamples()) {
        lateReverbBuffer.copyFrom(0, lateReverbDelaySamples, 
                                buffer, 0, 0, 
                                buffer.getNumSamples() - lateReverbDelaySamples);
        lateReverbBuffer.copyFrom(1, lateReverbDelaySamples, 
                                buffer, 1, 0, 
                                buffer.getNumSamples() - lateReverbDelaySamples);
    }
    
    // Process through Schroeder reverberator for late reverb
    schroederReverb->process(lateReverbBuffer.getWritePointer(0), 
                            lateReverbBuffer.getWritePointer(1), 
                            buffer.getNumSamples());
    
    // Combine everything: buffer (direct sound) + tempBuffer (early reflections) + lateReverbBuffer (late reverb)
    // Direct sound is already in the buffer from HRTF processing
    
    // Add early reflections
    for (int channel = 0; channel < 2; ++channel) {
        buffer.addFrom(channel, 0, tempBuffer, channel, 0, buffer.getNumSamples());
    }
    
    // Add late reverberation
    for (int channel = 0; channel < 2; ++channel) {
        buffer.addFrom(channel, 0, lateReverbBuffer, channel, 0, buffer.getNumSamples());
    }
    
    // Apply overall gain to prevent clipping
    buffer.applyGain(0.7f);
}

void BRIRProcessor::processBuffer(juce::AudioBuffer<float>& inputBuffer, juce::AudioBuffer<float>& outputBuffer)
{
    // Create a copy of the input buffer for processing
    juce::AudioBuffer<float> processingBuffer;
    processingBuffer.makeCopyOf(inputBuffer);
    
    // Process the buffer through the BRIR
    process(processingBuffer);
    
    // Copy the processed buffer to the output buffer
    outputBuffer.clear();
    
    // Make sure we have the right number of channels and samples
    int numChannels = juce::jmin(processingBuffer.getNumChannels(), outputBuffer.getNumChannels());
    int numSamples = juce::jmin(processingBuffer.getNumSamples(), outputBuffer.getNumSamples());
    
    // Copy channels one by one
    for (int channel = 0; channel < numChannels; ++channel) {
        outputBuffer.addFrom(channel, 0, processingBuffer, channel, 0, numSamples);
    }
}