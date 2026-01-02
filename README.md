# Doppler Radar Speed Measurement using HB100

## Overview
This project implements a low-cost Doppler radar system for measuring the speed of moving
objects using an HB100 X-band continuous-wave radar module. The system integrates
analog signal conditioning, high-speed data acquisition, and FFT-based signal processing
to estimate target speed in real time.

## System Architecture
- **Radar Sensor:** HB100 X-band Doppler radar (10.525 GHz)
- **Analog Front-End:** LM358-based amplifier and filtering stage
- **Data Acquisition:** Arduino (10 kHz sampling rate)
- **Signal Processing:** MATLAB FFT-based frequency analysis
- **Output:** Real-time speed estimation (km/h)

## How It Works
1. The HB100 radar detects motion and produces a Doppler IF signal.
2. The LM358 circuit amplifies and filters the weak IF signal.
3. The Arduino samples the signal at 10 kHz and streams data to MATLAB.
4. MATLAB performs FFT analysis and extracts the dominant Doppler frequency.
5. The Doppler frequency is converted to speed using the radar calibration constant.

## Tools & Technologies
- Arduino IDE
- MATLAB
- LM358 Op-Amp
- HB100 Doppler Radar
- Breadboard-based prototyping

## Results
The system successfully estimated the speed of moving objects such as human motion
and hand movement with reliable real-time visualization. Experimental results showed
good agreement with theoretical Doppler predictions.

## How to Run
1. Upload the Arduino sketch from the `code/arduino` folder.
2. Connect the HB100 and amplifier circuit as shown in `schematics`.
3. Run the MATLAB script in `code/matlab` to start real-time processing.
4. Move a target in front of the radar and observe the estimated speed.

## Future Improvements
- Direction detection using quadrature channels
- Adaptive noise filtering
- Embedded FFT implementation
- Outdoor vehicle testing
