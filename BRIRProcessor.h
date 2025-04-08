#pragma once

#include <JuceHeader.h>
#include <vector>
#include <array>
#include <cmath>

// Forward declaration
class AudioPluginAudioProcessor;

//==============================================================================
/**
 * BRIRProcessor class
 * Handles the simulation of Binaural Room Impulse Responses (BRIR) using:
 * 1. Direct sound (via HRTF)
 * 2. Early reflections (Image Source Method)
 * 3. Late reverberation (Schroeder Reverberator)
 */
class BRIRProcessor
{
public:
    BRIRProcessor(AudioPluginAudioProcessor& processor);
    ~BRIRProcessor();

    // Initialize BRIR processor with current sample rate
    void prepare(double sampleRate, int samplesPerBlock);
    
    // Update room parameters
    void setRoomDimensions(float length, float width, float height);
    void setReverbTime(float t60);
    void setSourcePosition(float x, float y, float z);
    void setListenerPosition(float x, float y, float z);
    void setWallAbsorption(float leftWall, float rightWall, float frontWall, 
                           float backWall, float ceiling, float floor);
    
    // Enable/disable BRIR processing
    void setEnabled(bool shouldBeEnabled) { isEnabled = shouldBeEnabled; }
    bool getEnabled() const { return isEnabled; }
    
    // Get room parameters
    float getRoomLength() const { return roomLength; }
    float getRoomWidth() const { return roomWidth; }
    float getRoomHeight() const { return roomHeight; }
    float getReverbTime() const { return reverbTime; }

    // Helper methods for coordinate conversion
    std::array<float, 3> centerToCornerCoordinates(const std::array<float, 3>& centerCoords) const;
    std::array<float, 3> cornerToCenterCoordinates(const std::array<float, 3>& cornerCoords) const;   

    // Methods to get positions in centered coordinates
    std::array<float, 3> getSourcePositionCentered() const;
    std::array<float, 3> getListenerPositionCentered() const;
    
    // Main process method for BRIR rendering
    void process(juce::AudioBuffer<float>& buffer);
    
    // Update BRIR impulse responses when parameters change
    void updateBRIR();
    
    // Process a buffer through the BRIR (real-time convolution)
    void processBuffer(juce::AudioBuffer<float>& inputBuffer, juce::AudioBuffer<float>& outputBuffer);

private:
    // Reference to the main processor
    AudioPluginAudioProcessor& processorRef;
    
    // Room parameters
    float roomLength = 11.2f;  // Default room dimensions from the paper
    float roomWidth = 9.2f;
    float roomHeight = 3.9f;
    float reverbTime = 0.5f;   // RT60 in seconds
    
    // Source and listener positions (in room coordinates)
    std::array<float, 3> sourcePosition = {5.6f, 4.6f, 1.5f};  // Default: center of room
    std::array<float, 3> listenerPosition = {5.6f, 4.6f, 1.5f}; // Default: center of room
    
    // Wall absorption coefficients (1 - reflection coefficient)
    std::array<float, 6> wallAbsorption = {0.3f, 0.3f, 0.3f, 0.3f, 0.3f, 0.3f};
    
    // Enable/disable BRIR processing
    bool isEnabled = false;
    
    // Sampling rate
    double sampleRate = 44100.0;
    
    float calculateReflectionCoefficient(int i, int j, int k);
    
    // Speed of sound (m/s)
    const float speedOfSound = 343.0f;
    
    // Maximum reflection order for image source model
    const int maxReflectionOrder = 2;
    
    // Image source model methods
    void calculateImageSources();
    void applyImageSourceMethod();
    float calculateDistance(const std::array<float, 3>& pos1, const std::array<float, 3>& pos2);
    
    // Structure to hold virtual source data
    struct VirtualSource {
        std::array<float, 3> position;  // 3D position
        float distance;                 // Distance to listener
        float delay;                    // Delay in samples
        float amplitude;                // Amplitude factor
        int reflectionOrder;            // Total reflection order (i+j+k)
        std::array<int, 3> reflectionIndices; // i, j, k reflection indices
        
        // Constructor
        VirtualSource(const std::array<float, 3>& pos, float dist, float del, float amp, int order, 
                     const std::array<int, 3>& indices)
            : position(pos), distance(dist), delay(del), amplitude(amp), 
              reflectionOrder(order), reflectionIndices(indices) {}
    };
    
    // Container for all virtual sources
    std::vector<VirtualSource> virtualSources;
    
    // Direct path and early reflections
    juce::AudioBuffer<float> directPathIR;
    juce::AudioBuffer<float> earlyReflectionsIR;
    
    // BRIR components
    juce::AudioBuffer<float> combinedBRIR;
    
    // Convolution engines for direct sound and early reflections
    juce::dsp::Convolution directConvolutionL;
    juce::dsp::Convolution directConvolutionR;
    juce::dsp::Convolution earlyConvolutionL;
    juce::dsp::Convolution earlyConvolutionR;
    
    // Schroeder reverberator components for late reverb
    // Four parallel comb filters feeding into two series all-pass filters
    struct SchroederReverb {
        struct CombFilter {
            std::vector<float> buffer;
            int bufferSize;
            int writePos;
            float feedback;
            
            CombFilter(float delayMs, float rt60, double sr)
            {
                bufferSize = static_cast<int>(delayMs * sr / 1000.0);
                buffer.resize(static_cast<size_t>(bufferSize), 0.0f);
                writePos = 0;
                // Calculate feedback coefficient for desired RT60
                feedback = std::pow(0.001f, delayMs / (rt60 * 1000.0f));
            }
            
            float process(float input)
            {
                size_t bufSize = buffer.size();
                size_t readPos = (static_cast<size_t>(writePos) + bufSize - static_cast<size_t>(bufferSize)) % bufSize;
                float output = buffer[readPos];
                buffer[static_cast<size_t>(writePos)] = input + output * feedback;
                writePos = (writePos + 1) % static_cast<int>(bufSize);
                return output;
            }
        };
        
        struct AllPassFilter {
            std::vector<float> buffer;
            int bufferSize;
            int writePos;
            float gain;
            
            AllPassFilter(float delayMs, float g, double sr)
            {
                bufferSize = static_cast<int>(delayMs * sr / 1000.0);
                buffer.resize(static_cast<size_t>(bufferSize), 0.0f);
                writePos = 0;
                gain = g;
            }
            
            float process(float input)
            {
                size_t bufSize = buffer.size();
                size_t readPos = (static_cast<size_t>(writePos) + bufSize - static_cast<size_t>(bufferSize)) % bufSize;
                float delayedSample = buffer[readPos];
                float temp = input + delayedSample * gain;
                buffer[static_cast<size_t>(writePos)] = temp;
                writePos = (writePos + 1) % static_cast<int>(bufSize);
                return delayedSample - gain * temp;
            }
        };
        
        // Four comb filters with prime-number-based delay times to avoid resonance patterns
        std::vector<CombFilter> combFiltersL;
        std::vector<CombFilter> combFiltersR;
        
        // Two all-pass filters in series to increase echo density
        std::vector<AllPassFilter> allPassFiltersL;
        std::vector<AllPassFilter> allPassFiltersR;
        
        SchroederReverb(float rt60, double sr)
        {
            // Initialize comb filters with different delay times
            // Using prime-number-based delay times to avoid resonances
            float combDelays[4] = {29.7f, 37.1f, 41.1f, 43.7f};
            for (int i = 0; i < 4; i++) {
                combFiltersL.emplace_back(combDelays[i], rt60, sr);
                combFiltersR.emplace_back(combDelays[i] * 1.05f, rt60, sr); // Slightly different for R channel
            }
            
            // Initialize all-pass filters
            float allPassDelays[2] = {5.0f, 1.7f};
            float allPassGain = 0.7f;
            for (int i = 0; i < 2; i++) {
                allPassFiltersL.emplace_back(allPassDelays[i], allPassGain, sr);
                allPassFiltersR.emplace_back(allPassDelays[i] * 1.05f, allPassGain, sr);
            }
        }
        
        void process(float* left, float* right, int numSamples)
        {
            for (int i = 0; i < numSamples; ++i) {
                // Process comb filters in parallel
                float combOutL = 0.0f;
                float combOutR = 0.0f;
                
                for (auto& comb : combFiltersL)
                    combOutL += comb.process(left[i]);
                for (auto& comb : combFiltersR)
                    combOutR += comb.process(right[i]);
                
                // Normalize comb filter outputs
                combOutL /= static_cast<float>(combFiltersL.size());
                combOutR /= static_cast<float>(combFiltersR.size());
                
                // Process all-pass filters in series
                float apOutL = combOutL;
                float apOutR = combOutR;
                
                for (auto& ap : allPassFiltersL)
                    apOutL = ap.process(apOutL);
                for (auto& ap : allPassFiltersR)
                    apOutR = ap.process(apOutR);
                
                // Write output
                left[i] = apOutL;
                right[i] = apOutR;
            }
        }
    };
    
    std::unique_ptr<SchroederReverb> schroederReverb;
    
    // Buffer for late reverberation processing
    juce::AudioBuffer<float> lateReverbBuffer;
    
    // Late reverberation onset time (ms)
    float lateReverbOnset = 80.0f;  // 80ms for late reverb to start
    
    // Working temporary buffers
    juce::AudioBuffer<float> tempBuffer;
    
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BRIRProcessor)
};