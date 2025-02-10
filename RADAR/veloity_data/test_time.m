clear all

% Configuration file
cfgFile = 'C:\University\Project\Configurations\5m.cfg';
cfgPort = "COM4";
dataPort = "COM3";

% Create sensor 
rdr = mmWaveRadar("TI AWR1642BOOST",...
     ConfigPort = cfgPort,...
     DataPort = dataPort, ...
    ConfigFile=cfgFile,...
    ReadMode="latest",...
    DetectionCoordinates="sensor spherical",...
    EnableDopplerGroups=false,...
    EnableRangeGroups=false,...
    RemoveStaticClutter=false);

% Allocate memory
detectionLog = cell(1,0); 
timeLog = zeros(1,0); 
currentTimeLog = zeros(1,0);

% Duration of recording in secondns
disp(posixtime(datetime(now, 'ConvertFrom', 'datenum')));
%starttime=posixtime(datetime(now, 'ConvertFrom', 'datenum'));
stopTime = 30; 
disp("STARTING");
% Record until stop time
while isempty(timeLog) || timeLog(end) < stopTime
    [detectionLog{end+1},timeLog(end+1)] = rdr();
    currentTimeLog(end +1) = posixtime(datetime(now, 'ConvertFrom', 'datenum'));
    
end

disp("done?")
save(fullfile(pwd,'TEST_TIME1.mat'), 'detectionLog', 'timeLog','currentTimeLog');