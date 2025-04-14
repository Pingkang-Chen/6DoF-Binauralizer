Plugin's UI:
<img width="949" alt="Screenshot 2025-04-08 at 10 21 38" src="https://github.com/user-attachments/assets/23be1072-627d-4742-ab91-2fa005ffeda6" />

Dynamic VAE System: Direct sound is modeled by convolving the signal with the HRTF corresponding to the relative position (r, θ, ϕ) between the virtual source and the listener. In dynamic playback, the listener's head orientation is tracked in real time, and the HRTF lookup and convolution are updated accordingly. Early reflections are simulated using a simplified image-source method. The first- and second-order reflections are treated as independent virtual sources. Given the room geometry and positions of the listener and source, the spatial coordinates of each image source are computed. Each reflection undergoes delay (Delayi) and attenuation (αi) to simulate wall absorption and distance-dependent energy decay. These reflections are then spatialized using corresponding HRTFs (HL,i, HR,i) and the outputs are summed for both ears. During dynamic playback, the image source positions are continuously recalculated based on real-time head-tracking data, ensuring accurate spatialization of reflections. Late reverberation is synthesized using a Schroeder network consisting of four parallel comb filters and two serial all-pass filters. To avoid coloration, the delay lengths in the comb filters are set to irregular, non-integer values. The reverberation tail beyond 80 ms is decorrelated for the left and right ears before being added to the binaural output, ensuring spatial realism. Unlike the direct sound and early reflections, the late reverberation is modeled as a diffuse sound field that lacks directional properties, depending only on the room's acoustic parameters rather than source or listener orientation.

1: You need to download the SOFA file used in this plugin through this link, and put it on your desktop before running this plugin:
https://sofacoustics.org/data/database/scut/SCUT_NF_subject0006_measured.sofa

2: Dependencies: JUCE, libmysofa

3: Build instructions: 

mkdir build

cd build

cmake ..

cmake --build .
