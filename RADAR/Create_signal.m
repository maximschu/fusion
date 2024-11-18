%By Honghao Tang 

function  [signal] = Create_signal(N,doa,w,M,P,lambda,d,snr)


format long %The data show that as long shaping scientific 

D=zeros(P,M); %To creat a matrix with P row and M column 
for k=1:P  
D(k,:)=exp(-j*2*pi*d*sin(doa(k))/lambda*[0:M-1]); %Assignment matrix 
end  
D=D';  
xx=2*exp(j*(w*[1:N])); %Simulate signal 
x=D*xx;   
x=x+awgn(x,snr);%Insert Gaussian white noise

signal = x;