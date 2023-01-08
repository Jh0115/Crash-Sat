clear all
close all
clc

I = 0.005;
D = 2;
phi = deg2rad(5:60);
s = 0.015:0.001:0.2

[s,phi] = meshgrid(s,phi);

F = (pi*I./s+D)./cos(phi);

r = 0.005;
P = F/(pi*r*r) %pressure in pascals

P = P/6895; %convert to PSI

surf(s,phi,P)
xlabel('span')
ylabel('angle')
