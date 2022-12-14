%% Pitch simulation - Crash-sat
clear all
close all
clc

%% Assumptions
% - Crash sat is nearly vertical when beginning pitch
% - Any wind, is constant or derived from a lookup table
% - Crash sat performs a constant G turn until horizontal with the ground

%% Rules
% - To track orientation, we will record the bdy frame x and y unit vector quaternions WRT the navigational frame
%   - Body-X extends out of the aircraft's nose
%   - Body-Y extends out of the aircraft's port wing
%   - Body-Z is cross(BX,BY)
%   - Origin is at COM where orientation occurs around
% - To track position we will use regular cartesian vectors.
%   - Navigational axis is North-East-Down, with the origin at ground level directly underneath the initial position of the body frame origin


%% Step 1: Initial conditions and data arrays
%POSE vectors
h = 4267; %initial height in meters
pBx0 = [1;0;0]; %initial orientation of body X unit vector in the navigational frame
pBy0 = [0;1;0]; %initial orientation of body Y unit vector in the navigational frame

%velocity vectors
v0 = 40; %initial speed in m/s
pV0 = [pBx0]; %initial orientation of velocity vector as a unit vector

%time vector
dt = 0.01;
t_end = 50;
t = 0:dt:t_end;

%% Step 1.5: dummy check ALL of the initial conditions to correct for stupid inputs and edge cases
s0 = [0,0,-h]; %initial position in coordinate frame North, East, Down
if v0<=0
  warning('Initial velocity cannot be set to zero or negative, defaulting to 1 m/s')
  v0 = 0.1;
endif

if w0<0
  warning('Initial angular velocity cannot be negative, defaulting to 0 rad/sec.')
  w0 = 0;
endif

pBx0 = normalize(pBx0);
pBy0 = normalize(pBy0);
pBz0 = cross(qBx0,qBy0);

s_arr = zeros(3,numel(t));
s_arr(:,1) = s0
v_arr = zeros(1,numel(t));
v_arr(1) = v0;
w_arr = zeros(1,numel(t));
w_arr(1) = w0;
pBx_arr = zeros(3,numel(t));
pBx_arr(:,1) = pBx0;
pBy_arr = zeros(3,numel(t));
pBy_arr(:,1) = pBy0;

%% Step 2: Primary loop (save true data, sourced from rungekutta method here
for ii = 1:numel(t)-1
%% Step 3: Secondary loop (run this 4 times each iteration to get runge-kutta coefficients
  for jj = 1:4
    %% Step 1: Using current orientation, calculate flight coefficients

    %% Step 2: Using height determine air desity and wind gusts

    %% Step 3: Using density, velocity, and orientation, calculate body frame forces

    %% Step 4: Using current orientation, calculate gravity forces in body frame

    %% Step 5: Using aerodynamic forces, control surfaces, velocity, and orientations, calculate moments in body frame

    %% Step 6: Using inertia matrix and moments, calculate angular velocity for next iteration

    %% Step 7: Using body frame forces determine velocity vector at next iteration
    kv = ;
    kw = ;

    RK4_coeff(:,jj) = [kv;kw];
  endfor

  % calculate velocity and angular velocity using runge kutta coefficients
  v_new = v_arr(ii)+(dt/6)*(RK4_coeff(1,1)+2*RK4_coeff(1,2)+2*RK4_coeff(1,3)+RK4_coeff(1,4));
  w_new = w_arr(ii)+(dt/6)*(RK4_coeff(2,1)+2*RK4_coeff(2,2)+2*RK4_coeff(2,3)+RK4_coeff(2,4));

%% Step 4: Updating data arrays
  v_arr(ii+1) = v_new;
  w_arr(ii+1) = w_new;

endfor

%% Step 5: Plot all data


