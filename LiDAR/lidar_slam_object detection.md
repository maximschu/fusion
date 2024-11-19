MATLAB code: --------note the paus 2 seconds, can be done much faster



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



results:![78c331f7-d4ac-47a8-a711-d8d321a3387a](https://github.com/user-attachments/assets/49f2d574-af7b-42da-934f-962056095956)
![57ff0690-edfd-448b-b4b8-17723e405d88](https://github.com/user-attachments/assets/9658ff1b-7ab1-4e94-9337-d3c1a7f76912)
![8c1def71-360e-4fc9-9bcd-174ce487e76d](https://github.com/user-attachments/assets/80190156-59c9-46c8-9a48-d48830d18ff1)
![682ba813-d509-423a-bb77-79501ef2e19a](https://github.com/user-attachments/assets/360e7192-7ecf-4288-b746-a7c8e7cecaf0)
![fc1ce32a-5ccf-444b-a0c6-8e56773682da](https://github.com/user-attachments/assets/9c040d07-9ee6-49a6-bf9d-bc3ab1c8c985)



Highlighted is figure above new "dynamic object" -> Below is 2seconds later, yellow on map labelled "diff" shows where the person is. 


 
