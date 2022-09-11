%% Radar Altimeter simulation
clear all
close all
clc

%% Input parameters
%State
speed = 0;
h = 80; %cannot be less than 1 meter

%Square wave circuit
V_square = 10;
noiseVar = 0;
f_square = 250000;

%VCO circuit
BW = 2000; %difference between high and low frequencies
k_VCO = 2*BW/V_square; %Hz/V
f_carrier = 450e6;
FMgain = 5; %peak voltage of FM waveform

%radar reflection plant
RXdir = 1.76; %directivity
TXdir = 1.76; %directivity
RXnoiseVar = 0; %variance of injected RX signal noise in millivolts

%% Design band pass filter
fh = f_carrier+BW/2;
fl = f_carrier-BW/2;
vmax = 300; %max possible velocity m/s
fdmax = (fh*3e8/(3e8-vmax))-fh;
fdmin = 0;

f1 = abs((fh)-(fl+fdmax)) %keep
f2 = abs((fh)-(fl+fdmin)) %keep
f3 = abs((fl)-(fh+fdmax)) %keep
f4 = abs((fl)-(fh+fdmin)) %keep

fh
fl
fdmax
fdmin

cutoff_low = 500;
cutoff_high = f_carrier;

RH = 100;
RL = 100;

CH = 1/(2*pi*RH*cutoff_low);
CL = 1/(2*pi*RL*cutoff_high);

BPFnum = [RH*CH,0];
BPFden = [RH*RL*CH*CL,RH*CH+RL*CL,1];

LPFnum = 1;
LPFden = [RL*CL,1];

LPFden = conv(LPFden,LPFden);
LPFden = conv(LPFden,LPFden);

%% Run simulink model
T_step = 0.0000000001;
simOut = sim("RadarAltimeter.slx",0.00001);

%% Post analysis
% f_sample = 1/T_step;
% 
% %put data into arrays
% sig = simOut.DAQ(2000:end);
% t = simOut.time(2000:end);
% 
% n = 1;
% rr = 2000;
% ii = 1;
% while n<numel(sig) && ii<=10000
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
