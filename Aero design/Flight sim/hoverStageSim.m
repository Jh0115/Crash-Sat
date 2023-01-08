clear all
close all
clc

%% hover sim state space model

%initial condition
v0 = 20; %initial m/s
h0 = 2; %initial meters
t_update = 2; %Seconds

dt = 0.025;
t_end = 10;
t = 0:dt:t_end
v = zeros(2,numel(t))
h = zeros(1,numel(t))
s = zeros(1,numel(t))

v(1,1) = v0;
h(1) = h0;

%loop
t_last_update = 0;

for ii = 1:numel(t)-1
  %update the ac_struct object given the new state values

  %recalculate taylor series linearizations every t_u seconds
  if t(ii)>(t_last_update+t_update)
    %update the linearizations

  %next decide what to do with control surface given controller and state model
  %no elevator controller for now
  elev = 0;

  %calculate the coefficients of flight given the current angle of attack

  %calculate the forces of flight given the coefficients and elevator angle

  %calculate the acting moment given the inherent vehicle moment and the elevator effect

  %knowing the forces and moments calculate the accelerations

  %using the accelerations determine the state at the next instance

endfor




