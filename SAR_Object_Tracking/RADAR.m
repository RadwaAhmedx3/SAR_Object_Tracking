clc; clear; close all;

%% ==========[ STEP 1: SAR IMAGE CONSTRUCTION ]==========
videoInput = VideoReader('CARS (2).mp4'); 
numFrames = floor(videoInput.FrameRate * videoInput.Duration);
frameRate = videoInput.FrameRate;

outputFileName = 'SAR_Signal_Controlled.avi';  
videoOutput = VideoWriter(outputFileName, 'Motion JPEG AVI');
videoOutput.FrameRate = frameRate;
open(videoOutput);

t = linspace(0, videoInput.Duration, numFrames);
signal = sin(2 * pi * 4 * t); 
frameCount = 0;

disp('⏳ Generating SAR video...');

for frameIdx = 1:numFrames
    if hasFrame(videoInput)
        frame = readFrame(videoInput);
        frameCount = frameCount + 1;

        grayFrame = rgb2gray(frame);
        sar_reflectivity = double(grayFrame) / 255;
        sar_modulated = sar_reflectivity * (1 + 0.7 * signal(frameCount));

        sar_raw_data = fft2(sar_modulated);
        sar_image = abs(ifft2(sar_raw_data));
        sar_image(isnan(sar_image)) = 0;

        sar_image_display = 20 * log10(sar_image + 1e-6);
        sar_image_display = mat2gray(sar_image_display);
        sar_frame = uint8(255 * sar_image_display);
        rgbFrame = repmat(sar_frame, [1, 1, 3]);

        writeVideo(videoOutput, rgbFrame);
    end
end

close(videoOutput);
disp('✅ SAR video saved and ready for tracking.');

%% ==========[ STEP 2: TARGET TRACKING ]==========
vidReader = VideoReader(outputFileName);
frame = readFrame(vidReader);


if size(frame, 3) == 3  
    grayFrame = rgb2gray(frame); 
    frame = cat(3, imadjust(grayFrame), imadjust(grayFrame), imadjust(grayFrame));  
else
    frame = imadjust(frame);  
end

figure('Position', [100, 100, 900, 700]); imshow(frame);
title('Select the object you want to track and press Enter', 'FontSize', 12, 'FontWeight', 'bold');
bbox = round(getrect); 
close;

tracker = vision.PointTracker('MaxBidirectionalError', 2, 'NumPyramidLevels', 3);

grayFrame = rgb2gray(frame);


points = detectMinEigenFeatures(grayFrame, 'ROI', bbox, 'MinQuality', 0.05);
initialize(tracker, points.Location, grayFrame);

kalmanFilter = configureKalmanFilter('ConstantVelocity', bbox(1:2), [2 2], [2 2], 2);


figure('Position', [50, 50, vidReader.Width, vidReader.Height]); 
videoAxes = axes;
hVideo = imshow(frame);
hold on;
rectangleHandle = rectangle('Position', bbox, 'EdgeColor', 'green', 'LineWidth', 2);


figure('Position', [1000, 100, 600, 500]); hold on;
title('Path Tracking', 'FontSize', 14, 'FontWeight', 'bold'); xlabel('X'); ylabel('Y');
axis([0 vidReader.Width 0 vidReader.Height]);
grid on;
set(gca, 'YDir', 'reverse'); 
trackPlot = plot(nan, nan, 'r', 'LineWidth', 2);
trackedPath = [];

frameSkip = 2; 
frameCount = 0;

while hasFrame(vidReader)
    frame = readFrame(vidReader);
    frameCount = frameCount + 1;
    
    if mod(frameCount, frameSkip) ~= 0
        continue;
    end
    
    
    if size(frame, 3) == 3  
        grayFrame = rgb2gray(frame);  
        frame = cat(3, imadjust(grayFrame), imadjust(grayFrame), imadjust(grayFrame));  
    else
        frame = imadjust(frame);  
    end
    
    grayFrame = rgb2gray(frame);
    [points, validity] = tracker(grayFrame);
    validPoints = points(validity, :);
    
    if ~isempty(validPoints)
        bbox = bboxFromPoints(validPoints);
        
        predictedLocation = predict(kalmanFilter);
        correctedLocation = correct(kalmanFilter, bbox(1:2));
        
        bbox(1:2) = correctedLocation;
        rectangleHandle.Position = bbox;
        rectangleHandle.Visible = 'on';
        
        centerX = bbox(1) + bbox(3) / 2;
        centerY = bbox(2) + bbox(4) / 2;
        trackedPath = [trackedPath; centerX, centerY];
        
        set(trackPlot, 'XData', trackedPath(:,1), 'YData', trackedPath(:,2));
    else
        disp('⚠️ Object lost! Attempting to reinitialize tracker...');
        
       
        tracker = vision.PointTracker('MaxBidirectionalError', 2, 'NumPyramidLevels', 3);
        points = detectMinEigenFeatures(grayFrame, 'ROI', bbox, 'MinQuality', 0.05);
        initialize(tracker, points.Location, grayFrame);
        rectangleHandle.Visible = 'off';  
        pause(1); 
    end
    
    set(hVideo, 'CData', frame);
    drawnow;
end

disp('🏁 Tracking finished.');

function bbox = bboxFromPoints(points)
    xMin = min(points(:, 1));
    yMin = min(points(:, 2));
    xMax = max(points(:, 1));
    yMax = max(points(:, 2));
    bbox = [xMin, yMin, xMax - xMin, yMax - yMin];
end
