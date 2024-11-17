clear all
close all
clc


%myFolder = strcat('C:\Users\Catherine\Desktop\Measurement\');       % Directory to stored captured file
%filespec = strcat('testrecording30CM.bin');                        % Captured file name
myFolder = strcat('C:\Users\Catherine\Desktop\Measurement\calibration_accom\');       % Directory to stored captured file
filespec = strcat('-25_200.bin');                        % Captured file name
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
%% MUSIC algorithm starts

signal = [range_time_matrix1(:,1), range_time_matrix2(:,1), range_time_matrix3(:,1),range_time_matrix4(:,1)];  %First chirp from each reciever
signal = signal';                           


correlation_matrix = (1/numADCSamples) * (signal*signal');      %Computing the correlation matrix of the received signal.


eigenvalues = eig(correlation_matrix);                      %Computing the eigenvalues and the eigenvectors of the correlation matrix.
[eigenvector,D] = eig(correlation_matrix);
[d,ind] = sort(diag(D));                                    %Sorting the eigenvalues and eigenvectors, extracting the eigenvectors correspond to the noises.
Ds = D(ind,ind);                                            %sorted eigenvalue
Vs = eigenvector(:,ind);                                    %sorted eigenvector

steering_vector = zeros(4, 181);
lambda = 3e8/77e9;
d = lambda/2;
%% 

angle = -90:1:90; 

eigenvector_noise = [Vs(:,1) Vs(:,2)];                       % Extracting the eigenvectors corresponding to the noises  

pseudo_spectrum = zeros(size(angle));                        % Creates empty psuedo spectrum matrix

for i = 1:length(angle)                                      %loop over the angle from -90 to 90 degrees in one degree increments
    theta = angle(i);

    steering_vector = exp(-1i * 2 * pi * (0:numRX-1) * (d * sind(theta) / lambda));             % Computing steering vector
    steering_vector = steering_vector';
    pseudo_spectrum(i) = 1/ (steering_vector' * (eigenvector_noise* eigenvector_noise')* steering_vector);    %Computing puesdo spectrum
end

figure
plot(angle, 10*log10(abs(pseudo_spectrum)));
xlabel('Angle (degrees)');
ylabel('Pseudo Spectrum');
title('MUSIC Pseudo Spectrum');

[pks,locs] = findpeaks(10*log10(abs(pseudo_spectrum)), angle,'MinPeakHeight', 0);         %Obtains angles at peak

fprintf('Detected Angle: %.2f  \n', locs);
