clear all
close all
clc

%% hover sim state space model

%initial condition
vy0 = 0; %initial m/s
vx0 = 20;
h0 = 20; %initial meters
th0 = deg2rad(5); %initial orientation
aoa0 = deg2rad(5); %initial angle of attack
w0 = deg2rad(0); %initial angular velocity
t_update = 2; %Seconds

dt = 0.025;
t_end = 10;
t = 0:dt:t_end
v = zeros(2,numel(t))
h = zeros(1,numel(t))
s = zeros(1,numel(t))

v(1,1) = vx0;
v(2,1) = vy0;
h(1) = h0;

%state vectorize
x = [h0;
     vy0;
     vx0;
     th0;
     w0;
     aoa0;];

%loop
t_last_update = 0;

for ii = 1:numel(t)-1
  %update the ac_struct object given the new state values

  %recalculate taylor series linearizations every t_u seconds
  if t(ii)>(t_last_update+t_update)
    %update the linearizations
    [f_vy,dvydh,dvydvy,dvydvx,dvydth,dvydw,dvyda] = [vy,0,1,0,0,0,0];
    [f_ay,daydh,daydvy,daydvx,daydth,daydw,dayda] = linearize_ay(ac_struct,h,vy,vx,th,w,a,rho,elev);
    [f_ax,daxdh,daxdvy,daxdvx,daxdth,daxdw,daxda] = linearize_ax(ac_struct,h,vy,vx,th,w,a,rho,elev);
    [f_w,dwdh,dwdvy,dwdvx,dwdth,dwdw,dwda] = [w,0,0,0,0,1,0];
    [f_alpha,dalphadh,dalphadvy,dalphadvx,dalphadth,dalphadw,dalphada] = linearize_alpha(ac_struct,h,vy,vx,th,w,a,rho,elev);
    [f_daoa,ddaoadh,ddaoadvy,ddaoadvx,ddaoadth,ddaoadw,ddaoada] = linearize_aoa_dot(ac_struct,h,vy,vx,th,w,a,rho,elev);

    A = [dvydh,    dvydvy,    dvydvx,    dvydth,    dvydw,    dvyda;
         daydh,    daydvy,    daydvx,    daydth,    daydw,    dayda;
         daxdh,    daxdvy,    daxdvx,    daxdth,    daxdw,    daxda;
         dwdh,     dwdvy,     dwdvx,     dwdth,     dwdw,     dwda;
         dalphadh, dalphadvy, dalphadvx, dalphadth, dalphadw, dalphada;
         ddaoadh,  ddaoadvy,  ddaoadvx,  ddaoadth,  ddaoadw,  ddaoada]; %state matrix

    B = [0,f_vy;
         0,f_ay;
         0,f_ax;
         0,f_w;
         0,f_alpha;
         0,f_daoa]; %input matrix (elevator and bias as columns)

    t_last_update = t(ii);

  endif

  %next decide what to do with control surface given controller and state model
  %no elevator controller for now
  elev = 0;
  u = [elev;
       1];

  %calculate the coefficients of flight given the current angle of attack

  %calculate the forces of flight given the coefficients and elevator angle

  %calculate the acting moment given the inherent vehicle moment and the elevator effect

  %knowing the forces and moments calculate the accelerations

  %using the accelerations determine the state at the next instance

endfor




