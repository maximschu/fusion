clear all
close all
clc


myFolder = strcat('C:\Users\Catherine\Desktop\Measurement\');       % Directory to stored captured file
filespec = strcat('testrecording100CM.bin');                        % Captured file name
prf = 4000;                                                         % PRF of waveform
slope = 3.7513e13;                                                  % slope of chirped signal
isReal = 0;                                                         % Set to 1 if real only data, 0 if complex data
numADCSamples = 256;                                                % Number of ADC samples per chirp
numADCBits = 16;                                                    % Number of ADC bits per sample
numRX = 4;                                                          % Number of receivers
numFiles = 1;                                                       % Number of total files used to store capture

sample_rate = 2500;                                                 % ksps

start_delay = 0;                                                    % Amount of time to cut off the beginning data (s)

fileName = fullfile(myFolder,filespec);


%% READING IN DATA
if numFiles == 1
    [retVal,total_chirps] = readDCA1000(fileName,isReal,numADCSamples,numADCBits,numRX);                        % Read in data for capture stored in single file
else
    [retVal,total_chirps] = multifile_readDCA1000(myFolder,isReal,numADCSamples,numADCBits,numRX,numFiles);     % Read in data for capture stored in multiple files
end
%% 
receiver1 = retVal(1,:);                                           % Seperating ADC data by receiver
receiver2 = retVal(2,:);                                           % rows of retVal = receiver number (total 4 receiver)
receiver3 = retVal(3,:);
receiver4 = retVal(4,:);
                                                                   % Retrieve data for current channel                
if numRX == 1
    receiver1 = receiver1(1:2:end);
    total_chirps = total_chirps/2;
end
raw_data_matrix1 = reshape(receiver1,[numADCSamples, total_chirps]);                                             % Reshape into fast/slow matrix
raw_data_matrix2 = reshape(receiver2,[numADCSamples, total_chirps]);                  
raw_data_matrix3 = reshape(receiver3,[numADCSamples, total_chirps]);                  
raw_data_matrix4 = reshape(receiver4,[numADCSamples, total_chirps]);                  

start_val = (start_delay*prf) + 1;                                   % Determine starting sample if inital delay required

raw_data_matrix1 = raw_data_matrix1(:,start_val:end);                % Crop data                     
raw_data_matrix2 = raw_data_matrix2(:,start_val:end);                                
raw_data_matrix3 = raw_data_matrix3(:,start_val:end);                                
raw_data_matrix4 = raw_data_matrix4(:,start_val:end);                                
%% Hanning Window and FFT and magnitude of signal
normalised_data_matrix1 = raw_data_matrix1 - mean(raw_data_matrix1);                      % Subtract mean value from all data samples - centres data around 0
normalised_data_matrix2 = raw_data_matrix2 - mean(raw_data_matrix2);                      % Subtract mean value from all data samples - centres data around 0
normalised_data_matrix3 = raw_data_matrix3 - mean(raw_data_matrix3);                      % Subtract mean value from all data samples - centres data around 0
normalised_data_matrix4 = raw_data_matrix4 - mean(raw_data_matrix4);                      % Subtract mean value from all data samples - centres data around 0


windowed_data_matrix1 = normalised_data_matrix1.*hann(size(normalised_data_matrix1,1));   % Taper columns using Hanning window - removes spectal leakage
windowed_data_matrix2 = normalised_data_matrix2.*hann(size(normalised_data_matrix2,1));   
windowed_data_matrix3 = normalised_data_matrix3.*hann(size(normalised_data_matrix3,1));   
windowed_data_matrix4 = normalised_data_matrix4.*hann(size(normalised_data_matrix4,1));   

range_time_matrix1 = fft(windowed_data_matrix1);                                         % Range FFT - fourier transform
range_time_matrix2 = fft(windowed_data_matrix2);                                     
range_time_matrix3 = fft(windowed_data_matrix3);                                     
range_time_matrix4 = fft(windowed_data_matrix4);                                  

magnitude_dB1 = 20*log10(abs(range_time_matrix1));                                       % converting complex to mangnitude in dB
magnitude_dB2 = 20*log10(abs(range_time_matrix2));                                 
magnitude_dB3 = 20*log10(abs(range_time_matrix3));                            
magnitude_dB4 = 20*log10(abs(range_time_matrix4)); 
%% Identifying object distance
[pks, locs] = findpeaks(magnitude_dB1(:,1),'MinPeakHeight', 73, 'MinPeakDistance',3.9 );    %Peak detection

f_s = sample_rate *1e3;                                             % Sampling frequency
f = (0:(numADCSamples-1)) * (f_s / numADCSamples);                  % Frequency axis
distances = (3e8 .* f) / (2 * slope);                               % Calculates distance

detected_frequencies = f(locs);                                     % Frequencies of detected peaks
distance_object = (3e8 .* detected_frequencies) / (2 * slope);      % Calculate distance for detected object

%% Plot of 1D FFT amplitude profile in freq
figure;
plot(f, magnitude_dB1(:, 1));                                       % Plot specified chirp num

xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Frequency vs Magnitude Spectrum');

grid on;
%% Plot of 1D FFT amplitude profile in distance with objects highlighted
figure;
plot(distances, magnitude_dB1(:, 1));                               % Plot specified chirp num

hold on;
plot(distance_object, pks, 'ro', 'MarkerFaceColor', 'r');           % overlay detected peaks 

xlabel('Distance (m)');
ylabel('Magnitude (dB)');
title('Distance vs Magnitude Spectrum');

grid on;

