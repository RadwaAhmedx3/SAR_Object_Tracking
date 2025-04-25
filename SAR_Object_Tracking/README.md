# SAR Object Tracking in MATLAB

This project generates a Synthetic Aperture Radar (SAR)-like video from a regular video using amplitude modulation at 4Hz, then tracks an object selected by the user.

## Features
- SAR Image Construction with FFT/IFFT
- 4Hz Sinewave Modulation (Amplitude 0.7)
- Object Tracking using PointTracker and Kalman Filter
- Path visualization of the tracked object

## How to Run
1. Make sure the video file `CARS (2).mp4` is in the same folder.
2. Run `main.m` script in MATLAB.
3. Select the object you want to track.
4. The SAR video will be generated and tracking will be displayed.

## Files
- `main.m` - Main script for SAR video generation and tracking.
- `CARS (2).mp4` - Input video.
- `SAR_Signal_Controlled.avi` - Output SAR video generated.

## Requirements
- MATLAB R2020b or newer
- Image Processing Toolbox
- Computer Vision Toolbox

## Author
[Team ODEyssey]
