function [retVal, numChirps] = readDCA1000(myFolder,isReal,numADCSamples,numADCBits,numRX,numFiles)

%% global variables
% change based on sensor config
numLanes = 2; % do not change. number of lanes is always 2

for part = 1:numFiles      
    identifier = part-1;
    filespec = strcat('\file1_',num2str(identifier),'.bin');
    fileName = fullfile(myFolder,filespec);
    %% read file
    % read .bin file
    fid = fopen(fileName,'r');
    partialadcData = fread(fid, 'int16');
    
    % if 12 or 14 bits ADC per sample compensate for sign extension
    if numADCBits ~= 16
        l_max = 2^(numADCBits-1)-1;
        partialadcData(partialadcData > l_max) = partialadcData(partialadcData > l_max) - 2^numADCBits;
    end
    fclose(fid);
    partialfileSize = size(partialadcData, 1);
    if part == 1
        adcData = transpose(partialadcData);
        fprintf('Size 1: %f, Size 2: %f\n', size(partialadcData,1), size(partialadcData,2))
        fileSize = partialfileSize;
    else
        adcData = [adcData transpose(partialadcData)];
        fprintf('Size 1: %f, Size 2: %f\n', size(partialadcData,1), size(partialadcData,2))
        fileSize = fileSize + partialfileSize;
    end
    clear('partialadcData');
end
adcData = transpose(adcData);    

% real data reshape, filesize = numADCSamples*numChirps
if isReal
    numChirps = fileSize/numADCSamples/numRX;
    fprintf('chirps: %f\n',numChirps)
    LVDS = zeros(1, fileSize);
    %create column for each chirp
    LVDS = reshape(adcData, numADCSamples*numRX, numChirps);
    %each row is data from one chirp
    LVDS = LVDS.';
else
    % for complex data
    % filesize = 2 * numADCSamples*numChirps
    numChirps = fileSize/2/numADCSamples/numRX;
    fprintf('chirps: %f\n',numChirps)
    LVDS = zeros(1, fileSize/2);
    %combine real and imaginary part into complex data
    %read in file: 2I is followed by 2Q
    counter = 1;
    for i=1:4:fileSize-1
        LVDS(1,counter) = adcData(i) + sqrt(-1)*adcData(i+2);
        LVDS(1,counter+1) = adcData(i+1)+sqrt(-1)*adcData(i+3);
        counter = counter + 2;
    end
    % create column for each chirp
    LVDS = reshape(LVDS, numADCSamples*numRX, numChirps);
    %each row is data from one chirp
    LVDS = LVDS.';
end
%organize data per RX
adcData = zeros(numRX,numChirps*numADCSamples);
for row = 1:numRX
    for i = 1:numChirps
        adcData(row, (i-1)*numADCSamples+1:i*numADCSamples) = LVDS(i, (row-1)*numADCSamples+1:row*numADCSamples);
    end
end
% return receiver data
retVal = adcData;
