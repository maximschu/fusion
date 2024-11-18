clc  
clear all  

numADCSamples = 200;

doa=[0 60]/180*pi; %Direction of arrival 
w=[pi/4 pi/3]';%Frequency 
numRX=4;%Number of array elements 
P=length(w); %The number of signal 

lambda=3e8/77e9;%Wavelength 
d=lambda/2;%Element spacing 
snr=20;%SNA 

[signal] = Create_signal(numADCSamples,doa,w,numRX,P,lambda,d,snr);



correlation_matrix = 1/numADCSamples * (signal*signal');      %Computing the correlation matrix of the received signal.


eigenvalues = eig(correlation_matrix);                      %Computing the eigenvalues and the eigenvectors of the correlation matrix.
[eigenvector,D] = eig(correlation_matrix);
[d,ind] = sort(diag(D));                                    %Sorting the eigenvalues and eigenvectors, extracting the eigenvectors correspond to the noises.
Ds = D(ind,ind);   %sorted eigenvalue
Vs = eigenvector(:,ind);      %sorted eigenvector

steering_vector = zeros(4, 181);
lambda = 3e8/77e9;
d = lambda/2;
%% 


angle = -90:1:90; 

eigenvector_noise = [Vs(:,1) Vs(:,2)];

pseudo_spectrum = zeros(size(angle));

for i = 1:length(angle)                      %loop over the angle from -90 to 90 degrees in one degree increments
    theta = angle(i);

    steering_vector = exp(-1i * 2 * pi * (0:numRX-1) * (d * sind(theta) / lambda));
    steering_vector = steering_vector';
    pseudo_spectrum(i) = 1/ (steering_vector' * (eigenvector_noise* eigenvector_noise')* steering_vector);    %Computing puesdo spectrum
    %pseudo_spectrum(i) = 1/ norm(eigenvector_noise'.* steering_vector)^2;
    %pseudo_spectrum(i) = 1 / norm(eigenvector_noise' * steering_vector)^2;
end
figure
plot(angle, 10*log10(abs(pseudo_spectrum)));
xlabel('Angle (degrees)');
ylabel('Pseudo Spectrum');
title('MUSIC Pseudo Spectrum');