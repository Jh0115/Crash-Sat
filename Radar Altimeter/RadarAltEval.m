%% Radar Altimeter simulation
clear all
close all
clc
warning('off')

%% Input parameters
%State
speed = 0;
h = 20; %cannot be less than 1 meter

%Square wave circuit
V_square = 10;
noiseVar = 0;
f_square = 500000;

%Integrator circuit
R1 = 100000;
R2 = 300;
C1 = 0.00000156;

%HPF circuit
R3 = 100;
C2 = 4.7e-6;

%VCO circuit
BW = 1000; %difference between high and low frequencies
k_VCO = 2*BW/V_square; %Hz/V
f_carrier = 1e9;
FMgain = 5; %peak voltage of FM waveform

%radar reflection plant
RXdir = 1.76; %directivity
TXdir = 1.76; %directivity
RXnoiseVar = 0; %variance of injected RX signal noise in millivolts

%LPF circuit
R4 = 10000;
C3 = 4.7e-9;

disp('LPF cutoff:')
disp(1/(2*pi*R4*C3))
disp('')

disp('HPF cutoff:')
disp(1/(2*pi*R3*C2))
disp('')
%% Run simulink model
T_step = 0.0000001;
simOut = sim("RadarAltimeter.slx",0.001);

% %% Post analysis
% f_sample = 1/T_step;
% 
% %put data into arrays
% sig = simOut.DAQ(2000:end);
% t = simOut.time(2000:end);
% 
% n = 1;
% rr = 20000;
% ii = 1;
% while n<numel(sig) && ii<=1000
%     sigNew(ii) = sig(n);
%     tnew(ii) = t(n);
%     ii = ii+1;
%     n = n+rr;
% end
% f_sample = 1/(tnew(2)-tnew(1));
% sig = sigNew;
% 
% Y = 2*abs(fft(sig)/numel(sig));
% Y = Y(1:end/2);
% f = linspace(0,(f_sample/2),numel(Y));
% 
% figure()
% plot(f,Y)
% grid()


