%% Pitch simulation - Crash-sat
clear all
close all
clc

%% Assumptions
% - Crash sat is nearly vertical when beginning pitch
% - Any wind, is constant
% - Crash sat performs a constant G turn until horizontal with the ground
cd = 0.3;
sa = 0.05;
b = sa*cd*1.225/2;
m = 10;

accel = @(h,v) 9.81+(b/m*(123.75*(-v)-0.45*(h)+3328));

h0 = -5000;
v0 = 1;

dt = 0.1;
t = 0:dt:50; %30 seconds

h_arr = zeros(1,numel(t));
v_arr = zeros(1,numel(t));

h_arr(1) = h0;
v_arr(1) = v0;

for ii = 1:numel(t)-1
  a = accel(h_arr(ii),v_arr(ii));
  dv = a*dt;

  v_arr(ii+1) = v_arr(ii)+dv;
  dh = v_arr(ii)*dt;
  h_arr(ii+1) = h_arr(ii)+dh;
endfor



figure()
subplot(2,1,1)
plot(t,v_arr)
title("State Variables For 1-Dimensional Falling Simulation")
ylabel("Velocity (m/s)")
grid()
subplot(2,1,2)
plot(t,h_arr)
ylabel("Height (m)")
xlabel("Time (sec)")
grid()


%now run a non-approximated function

r = @(h) 1.225*(-8e-5*h+0.993);

h = zeros(1,numel(t));
v = zeros(1,numel(t));

h(1) = -h0;
v(1) = -v0;

for ii = 1:numel(t)-1
  rho = r(h(ii));
  drag = (0.5*sa*cd*rho*v(ii)^2)/m;
  a = drag-9.81;

  dv = a*dt;
  v(ii+1) = v(ii)+dv;
  dh = v(ii)*dt;
  h(ii+1) = h(ii)+dh;

  v_term(ii) = sqrt(2*m*9.81/(rho*sa*cd));
endfor

h = -h;
v = -v;

subplot(2,1,1)
hold on
plot(t,v)
plot(t(1:end-1),v_term,"--k")
subplot(2,1,2)
hold on
plot(t,h)
ylim([-5000,0])







