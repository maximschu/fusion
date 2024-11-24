# Setup 
1.) Goto https://www.slamtec.com/en/Support#rplidar-a-series


2.)Install Driver for the USB AdapterThe USB adapter converts UART to USB by using CP2102 chip. You need to installthe device driver for the chip. The driver can be found in the provided SDKpackage or downloaded from Silicon Labs' official website:http://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspxHere's the installation steps in Windows: after connecting the RPLIDAR with PC,please find the driver file "CP210x VCP Windows" and choose correct operatingsystem version accordingly: x86 for 32-bit OS and x64 for 64-bitOS


3.) You can now run tge Robo Studio Demo (https://www.manualslib.com/manual/1760808/Slamtec-Rplidar-A2-Series.html?page=7#manual) Follow the steps on here. 

4.) Other codes can be found here https://github.com/slamtec/rplidar_sdk#
-Follow the steps to extract the workspace then *in Visual Studio* build the code


Then run the examples:
 C:\Users\maksy\Desktop\LIDAR\rplidar_sdk-master\output\x64\Debug> .\ultra_simple --channel --serial \\.\COM3 115200

outputs:
theta: 333.41 Dist: 00000.00
   theta: 333.72 Dist: 00524.00
   theta: 334.31 Dist: 00522.00
   theta: 334.86 Dist: 00522.00
   theta: 335.39 Dist: 00520.00
   theta: 335.93 Dist: 00518.00
   theta: 336.47 Dist: 00514.00
   theta: 337.08 Dist: 00512.00
   theta: 337.60 Dist: 00512.00
   theta: 338.15 Dist: 00511.00
   theta: 338.69 Dist: 00509.00
   theta: 339.23 Dist: 00507.00
   theta: 339.76 Dist: 00505.00
   theta: 340.31 Dist: 00502.00
   theta: 340.91 Dist: 00501.00


 Or run 
 PS C:\Users\maksy\Desktop\LIDAR\rplidar_sdk-master\output\x64\Debug> .\frame_grabber --channel --serial \\.\COM3 115200




# Object detection?
Before this please check slam ros for steps on connecting LiDAR to MATLAB (https://github.com/maximschu/fusion/blob/main/LiDAR/slam%20ros)


## currrent optimised code can be found at optimised_LiDAR_object_detection_matlab

## MATLAB code: --------note the pause is 2 seconds, can be done much faster


**
maxLidarRange = 8; % Maximum range of your LiDAR sensor in meters
mapResolution = 20; % Resolution of the map (cells per meter)

slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  % Adjust based on environment
slamAlg.LoopClosureSearchRadius = 8; % Maximum radius for loop closure

maxIterations = 10; % Adjust based on the number of scans you have

for i = 1:maxIterations
    scanMsg = receive(sub, 10); % Receive each new scan message
    ranges = double(scanMsg.Ranges); % Get the range data
    % Generate angles to match the number of ranges
    numAngles = numel(ranges); % Get the number of range measurements
    angles = linspace(double(scanMsg.AngleMin), double(scanMsg.AngleMax), numAngles);
    % Create the lidarScan object
    scan = lidarScan(ranges, angles);
    % Add the scan to the SLAM algorithm
    [isAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    % Optional: Visualize the SLAM process
    figure();
    title("slamalg")
    show(slamAlg);
    pause(0.01); % Pause briefly for visualization
    [rawScans, rawPoses] = scansAndPoses(slamAlg);
    rawPoints = [];
    currentPose = rawPoses(i, :);  % Extract current pose as [x, y, theta]
    % Transform scan to world frame
    scan = transformScan(rawScans{i}, currentPose);
    % Concatenate together
    rawPoints = [rawPoints; scan.Cartesian];
    [scans, poses] = scansAndPoses(slamAlg);
    map = buildMap(scans, poses, mapResolution, maxLidarRange);
    rawPointsMap = binaryOccupancyMap(map.XWorldLimits(2) - map.XWorldLimits(1), ...
                                  map.YWorldLimits(2) - map.YWorldLimits(1), ...
                                  map.Resolution);

% Set the grid origin to match the existing map
rawPointsMap.GridLocationInWorld = map.GridLocationInWorld;

% Mark cells occupied by the raw points
setOccupancy(rawPointsMap, rawPoints, 1);  % Mark occupied cells


% Change map format to matrices
rawMatrix = occupancyMatrix(rawPointsMap);
mapMatrix = occupancyMatrix(map);

% Ensure matrices are the same size
sizeRaw = size(rawMatrix);
sizeFinal = size(mapMatrix);

if ~isequal(sizeRaw, sizeFinal)
    maxSize = max(sizeRaw, sizeFinal);
    rawMatrix = padarray(rawMatrix, maxSize - sizeRaw, NaN, 'post');
    mapMatrix = padarray(mapMatrix, maxSize - sizeFinal, NaN, 'post');
end

% diff
diffMatrix = rawMatrix - mapMatrix;

% show diff
figure();
imagesc(diffMatrix);
colorbar;
title('Diff');
pause(2);
end
**


## results:
3s->5s
![78c331f7-d4ac-47a8-a711-d8d321a3387a](https://github.com/user-attachments/assets/49f2d574-af7b-42da-934f-962056095956)
5s->7s
![57ff0690-edfd-448b-b4b8-17723e405d88](https://github.com/user-attachments/assets/9658ff1b-7ab1-4e94-9337-d3c1a7f76912)
7s->9s
![8c1def71-360e-4fc9-9bcd-174ce487e76d](https://github.com/user-attachments/assets/80190156-59c9-46c8-9a48-d48830d18ff1)
9s->11s
![682ba813-d509-423a-bb77-79501ef2e19a](https://github.com/user-attachments/assets/360e7192-7ecf-4288-b746-a7c8e7cecaf0)
15->17s
![fc1ce32a-5ccf-444b-a0c6-8e56773682da](https://github.com/user-attachments/assets/9c040d07-9ee6-49a6-bf9d-bc3ab1c8c985)



Highlighted is figure above new "dynamic object" -> Below is 2seconds later, yellow on map labelled "diff" shows where the person is. 

Note that in 15->17s no object is moving therofre no yellow is identified
 
Current optimised code is at optimised_LiDAR_object_detection_matlab 

do not do more than 208 figures as that is matlab limit.

zip file with large test of 250 images attatched under large_test_zip

