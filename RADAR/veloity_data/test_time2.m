close all

load(fullfile(pwd,'TESTING.mat'),'detectionLog','timeLog','currentTimeLog');
detectionLog = detectionLog';
%startingtime=starttime;
display = HelperTIRadarTrackingDisplay('XLimits',[0 6],...
    'YLimits',[-3 3],...
    'MaxRange',25,...
    'PlotReferenceImage', false, ...
    'RadarReferenceLines',zeros(2,0));

%refImage = read(vidReader, 140);
display(detectionLog{1}, {}, objectTrack.empty(0,1), []);

for k = 1:numel(detectionLog)
    detections = detectionLog{k};
    for j = 1:numel(detections)
        detections{j}.MeasurementNoise(1,1) = 30; % 15^2 deg^2
        detections{j}.MeasurementNoise(2,2) = 0.046^2; % 0.6^2 m^2
    end
    detectionLog{k} = detections;
end

trackingData = zeros(0, 7, "double");

minRangeRate = 0.01;

epsilon = 4;
minNumPts = 1;
tracker = trackerJPDA(TrackLogic="Integrated");
tracker.FilterInitializationFcn = @initPeopleTrackingFilter;

% Volume of measurement space
azSpan = 60;
rSpan = 5.09;
dopplerSpan = 5;
V = azSpan*rSpan*dopplerSpan;

% Number of false alarms per step
nFalse = 5;

% Number of new targets per step
nNew = 0.1;

% Probability of detecting the object
Pd = 0.9;

tracker.ClutterDensity = nFalse/V;
tracker.NewTargetDensity = nNew/V;
tracker.DetectionProbability = Pd;

% Confirm a track with more than 95 percent
% probability of existence
tracker.ConfirmationThreshold = 0.95; 

% Delete a track with less than 0.0001
% probability of existence
tracker.DeletionThreshold = 1e-4;

tracks = objectTrack.empty(0,1);

% Initialize a container for velocity and position data (index, time, velocity, ID, position)
%trackingData = {};  % Will store rows of [index, time, velocity_x, velocity_y, position_x, position_y, ID]

for k = 1:numel(detectionLog)
    % Timestamp
    time = timeLog(k);
    time_date = double(currentTimeLog(k));
    %disp(num2str(time_date))
    
    % Radar at current time stamp
    detections = detectionLog{k};
    
    % Remove static returns
    %isDynamic = false(1,numel(detections));
    %for d = 1:numel(detections)
    %    isDynamic(d) = abs(detections{d}.Measurement(3)) > minRangeRate;
    %end
    %detectionsDynamic = detections(isDynamic);

    % Camera image
    % refImage = read(vidReader, k);

    % Cluster detections
    if isempty(detections)
        clusters = zeros(0,1,'uint32');
    else
        clusters = partitionDetections(detections, epsilon, minNumPts, 'Algorithm', 'DBSCAN');
    end
    
    % Centroid estimation
    clusteredDets = mergeDetections(detections, clusters);

    % Track centroid returns
    if isLocked(tracker) || ~isempty(clusteredDets)
        tracks = tracker(clusteredDets, time);
    end

    % Store velocity and position data for all tracks
    for i = 1:length(tracks)
        track = tracks(i);
        
        % Extract velocity and position from the track state (state = [position_x, velocity_x, position_y, velocity_y])
        velocity_x = double(track.State(2));  % Velocity in x-direction (state(2))
        velocity_y = track.State(4);  % Velocity in y-direction (state(4))
        position_x = track.State(1);  % Position in x-direction (state(1))
        position_y = track.State(3);  % Position in y-direction (state(3))
        %disp(class(track.TrackID))

        %disp(class(velocity_x))
        %disp("vx")
        dTrackID = double(track.TrackID);
        % Append new row for each track at each time step (without checking for duplicates)
        trackingData = [trackingData; dTrackID, time_date, velocity_x, velocity_y, position_x, position_y,sqrt(velocity_x^2+velocity_y^2),atan2(velocity_x,-velocity_y)*360/(2*pi)];  % Add new row
    end

    % Update display
    display(detections, clusteredDets, tracks, []);

    if abs((time - 15)) <= 0.05
        im = getframe(gcf);
    end
end



function filter = initPeopleTrackingFilter(detection)
% Create 3-D filter first
filter3D = initcvekf(detection);

% Create 2-D filter from the 3-D
state = filter3D.State(1:4);
stateCov = filter3D.StateCovariance(1:4,1:4);

% Reduce uncertainty in cross range-rate to 5 m/s
velCov = stateCov([2 4],[2 4]);
[v, d] = eig(velCov);
D = diag(d);
D(2) = 1;
stateCov([2 4],[2 4]) = v*diag(D)*v';

% Process noise in a slowly changing environment
Q = 0.25*eye(2);

filter = trackingEKF(State = state,...
    StateCovariance = stateCov,...
    StateTransitionFcn = @constvel,...
    StateTransitionJacobianFcn = @constveljac,...
    HasAdditiveProcessNoise = false,...
    MeasurementFcn = @cvmeas,...
    MeasurementJacobianFcn = @cvmeasjac,...
    ProcessNoise = Q,...
    MeasurementNoise = detection.MeasurementNoise);

end





% If your original trackingData is an N-by-8 numeric matrix:
% Columns = [ID, Time, VelX, VelY, PosX, PosY, ResultantVel, Time2]

% Define the columns you want to keep, for instance: 1,2,5,6,7,8
% (i.e., ID, Time, PosX, PosY, ResultantVel, theta)
colsToKeep = [1, 2, 5, 6, 7, 8];

% Create a new matrix without VelX and VelY:
newData = trackingData(:, colsToKeep);

% Pick appropriate variable names for the *remaining* columns:
colNames = {'ID','t','x','y','Velocity','Angle'};

% Convert to a table
T = array2table(newData, 'VariableNames', colNames);

% Finally, write the table to CSV
writetable(T, 'RADAR.csv');
