clear all
close all
clc

%% hover sim state space model

%initial condition
vy0 = 0; %initial m/s
vx0 = 100;
h0 = 20; %initial meters
th0 = atan2(vy0,vx0)+deg2rad(3);%deg2rad(0); %initial orientation
w0 = deg2rad(0); %initial angular velocity
t_update = 2; %Seconds
rho = 1.225; %air density kg/m^3
mu = 0.0000181; %air viscocity in kg/(m-s)
w_damp = 0.1; %angular velocity dampener constant

dt = 0.01;
t_end = 60;
t = 0:dt:t_end;
v = zeros(2,numel(t));
h = zeros(1,numel(t));
s = zeros(1,numel(t));

v(1,1) = vx0;
v(2,1) = vy0;
h(1) = h0;
w(1) = w0;
th(1) = th0;


O = [cos(th0),sin(th0)];
V = [vx0,vy0];
aoa(1) = calculateAOA(O,V);

%initialize the design of the vehicle
ac_struct = initializeDesign();
LT_alpha_ref = ac_struct.alpha; %reference for alpha values of lookup tables
LT_vel_ref = ac_struct.vel; %reference for vel values of lookup tables

%state vectorize
x = [h0;
     vy0;
     vx0;
     th0;
     w0;
     aoa(1)];

%nonlinear simulation loop
t_last_update = 0;

for ii = 1:numel(t)-1
  %update the ac_struct object given the new state values
  %start by finding the closest relevant AOA and vel for the lookup tables
  spd = sqrt(x(2)^2+x(3)^2);
  alpha_ind = findClosest1D(LT_alpha_ref,x(6));
  vel_ind = findClosest1D(LT_vel_ref,spd);

  energy(ii) = ac_struct.m*9.81*x(1)+0.5*ac_struct.m*spd*spd+0.5*ac_struct.MOI_y*x(5)*x(5);

  %recalculate taylor series linearizations every t_u seconds
##  if t(ii)>(t_last_update+t_update)
##    %update the linearizations
##    f0_vy = x(2); dvydh = 0; dvydvy = 1; dvydvx = 0; dvydth = 0; dvydw = 0; dvyda = 0;
##
##    [trash,f0_ay,daydh,daydvy,daydvx,daydth,daydw,dayda] = linearize_ay(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##    [trash,f0_ax,daxdh,daxdvy,daxdvx,daxdth,daxdw,daxda] = linearize_ax(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##
##    f0_w = x(5); dwdh = 0; dwdvy = 0; dwdvx = 0; dwdth = 0; dwdw = 1; dwda = 0;
##
##    [trash,f0_alpha,dalphadh,dalphadvy,dalphadvx,dalphadth,dalphadw,dalphada] = linearize_alpha(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##    [trash,f0_daoa,ddaoadh,ddaoadvy,ddaoadvx,ddaoadth,ddaoadw,ddaoada] = linearize_aoa_dot(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##
##    A = [dvydh,    dvydvy,    dvydvx,    dvydth,    dvydw,    dvyda;
##         daydh,    daydvy,    daydvx,    daydth,    daydw,    dayda;
##         daxdh,    daxdvy,    daxdvx,    daxdth,    daxdw,    daxda;
##         dwdh,     dwdvy,     dwdvx,     dwdth,     dwdw,     dwda;
##         dalphadh, dalphadvy, dalphadvx, dalphadth, dalphadw, dalphada;
##         ddaoadh,  ddaoadvy,  ddaoadvx,  ddaoadth,  ddaoadw,  ddaoada]; %state matrix
##
##    B = [0,f0_vy;
##         0,f0_ay;
##         0,f0_ax;
##         0,f0_w;
##         0,f0_alpha;
##         0,f0_daoa]; %input matrix (elevator and bias as columns)
##
##    t_last_update = t(ii);
##
##  endif

  %next decide what to do with control surface given controller and state model
  %no elevator controller for now
  elev = 0;
  u = [elev;
       1];

  %=============================================================================
  %% RK4 method k1
  %calculate the coefficients of flight given the current angle of attack
  Cl = interpolateCoefficient(ac_struct,ac_struct.Cl,alpha_ind,vel_ind,x(6),spd);
  [cf_lam,cf_turb,Cf_Sa] = coeff_friction(spd,ac_struct.c,rho,mu,ac_struct.Sa);
  Cd = cf_lam+cf_turb+interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x(6),spd);
  Cd_induced = Cl*Cl/(pi*ac_struct.AR*ac_struct.oe);
  Cm = ac_struct.dCmda(alpha_ind,vel_ind)*x(6)+ac_struct.Cm0(alpha_ind,vel_ind);

  %calculate the forces of flight given the coefficients and elevator angle
  q_inf = 0.5*rho*spd*spd; %dynamic pressure

  Sa_actual = abs((ac_struct.Sa_top-ac_struct.Sa_front)*sin(x(6))+ac_struct.Sa_front); %the surface area can vary depending on the angle of attack
  FD_fric = q_inf*Cf_Sa; %friction drag force
  FD_press = q_inf*interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x(6),spd)*Sa_actual; %pressure drag force
  FD_induced = q_inf*Cd_induced*ac_struct.Sa;

  q_inf = 0.5*rho*spd*spd; %dynamic pressure
  FL = q_inf*Cl*ac_struct.Sa; %magnitude of lift force in newtons
  FD = FD_fric+FD_press+FD_induced; %magnitude of drag force in newtons

  phi = x(4)-x(6); %the velocity vector angle

  Lx = -FL*sin(phi);
  Ly = FL*cos(phi);

  Dx = -FD*cos(phi);
  Dy = -FD*sin(phi);

  %calculate the acting moment given the inherent vehicle moment and the elevator effect
  M = q_inf*Cm*ac_struct.Sa*ac_struct.c; %moment on craft in N-m

  %knowing the forces and moments calculate the accelerations
  ax = (Dx+Lx)/ac_struct.m;
  ay = ((Dy+Ly)/ac_struct.m)-9.81;
  ang_vel_damp = w_damp*x(5)*x(5);
  if x(5)>0
    ang_vel_damp = -ang_vel_damp;
  endif
  ang_accel = (M)/ac_struct.MOI_y+ang_vel_damp;

  %k1 vector
  k1 = [x(2);
        ay;
        ax;
        x(5);
        ang_accel];

  x_k1 = [(x(1)+dt*k1(1)/2);
          (x(2)+dt*k1(2)/2);
          (x(3)+dt*k1(3)/2);
          (x(4)+dt*k1(4)/2);
          (x(5)+dt*k1(5)/2);
          0];

  O = [cos(x_k1(4)),sin(x_k1(4))];
  V = [x_k1(3),x_k1(2)];

  x_k1(6) = calculateAOA(O,V); %AoA_k1;

  %% RK4 method k2

  spd = sqrt(x_k1(2)^2+x_k1(3)^2);
  alpha_ind = findClosest1D(LT_alpha_ref,x_k1(6));
  vel_ind = findClosest1D(LT_vel_ref,spd);

  Cl = interpolateCoefficient(ac_struct,ac_struct.Cl,alpha_ind,vel_ind,x_k1(6),spd);
  [cf_lam,cf_turb,Cf_Sa] = coeff_friction(spd,ac_struct.c,rho,mu,ac_struct.Sa);
  Cd = cf_lam+cf_turb+interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k1(6),spd);
  Cd_induced = Cl*Cl/(pi*ac_struct.AR*ac_struct.oe);
  Cm = ac_struct.dCmda(alpha_ind,vel_ind)*x_k1(6)+ac_struct.Cm0(alpha_ind,vel_ind);

  %calculate the forces of flight given the coefficients and elevator angle
  q_inf = 0.5*rho*spd*spd; %dynamic pressure

  Sa_actual = abs((ac_struct.Sa_top-ac_struct.Sa_front)*sin(x_k1(6))+ac_struct.Sa_front); %the surface area can vary depending on the angle of attack
  FD_fric = q_inf*Cf_Sa; %friction drag force
  FD_press = q_inf*interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k1(6),spd)*Sa_actual; %pressure drag force
  FD_induced = q_inf*Cd_induced*ac_struct.Sa;

  q_inf = 0.5*rho*spd*spd; %dynamic pressure
  FL = q_inf*Cl*ac_struct.Sa; %magnitude of lift force in newtons
  FD = FD_fric+FD_press+FD_induced; %q_inf*Cd*ac_struct.Sa; %magnitude of drag force in newtons

  phi = x_k1(4)-x_k1(6); %the velocity vector angle

  Lx = -FL*sin(phi);
  Ly = FL*cos(phi);

  Dx = -FD*cos(phi);
  Dy = -FD*sin(phi);

  %calculate the acting moment given the inherent vehicle moment and the elevator effect
  M = q_inf*Cm*ac_struct.Sa*ac_struct.c; %moment on craft in N-m

  %knowing the forces and moments calculate the accelerations
  ax = (Dx+Lx)/ac_struct.m;
  ay = ((Dy+Ly)/ac_struct.m)-9.81;
  ang_vel_damp = w_damp*x_k1(5)*x_k1(5);
  if x_k1(5)>0
    ang_vel_damp = -ang_vel_damp;
  endif
  ang_accel = (M)/ac_struct.MOI_y+ang_vel_damp;

  %k1 vector
  k2 = [x(2);
        ay;
        ax;
        x(5);
        ang_accel];

  x_k2 = [(x(1)+dt*k2(1)/2);
          (x(2)+dt*k2(2)/2);
          (x(3)+dt*k2(3)/2);
          (x(4)+dt*k2(4)/2);
          (x(5)+dt*k2(5)/2);
          0];

  %AoA_k2 = x_k2(4)-atan2(x_k2(2),x_k2(3));
  O = [cos(x_k2(4)),sin(x_k2(4))];
  V = [x_k2(3),x_k2(2)];

  x_k2(6) = calculateAOA(O,V); %AoA_k2;

  %% RK4 method k3

  spd = sqrt(x_k2(2)^2+x_k2(3)^2);
  alpha_ind = findClosest1D(LT_alpha_ref,x_k2(6));
  vel_ind = findClosest1D(LT_vel_ref,spd);

  Cl = interpolateCoefficient(ac_struct,ac_struct.Cl,alpha_ind,vel_ind,x_k2(6),spd);
  [cf_lam,cf_turb,Cf_Sa] = coeff_friction(spd,ac_struct.c,rho,mu,ac_struct.Sa);
  Cd = cf_lam+cf_turb+interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k2(6),spd);
  Cd_induced = Cl*Cl/(pi*ac_struct.AR*ac_struct.oe);
  Cm = ac_struct.dCmda(alpha_ind,vel_ind)*x_k2(6)+ac_struct.Cm0(alpha_ind,vel_ind);

  %calculate the forces of flight given the coefficients and elevator angle
  q_inf = 0.5*rho*spd*spd; %dynamic pressure

  Sa_actual = abs((ac_struct.Sa_top-ac_struct.Sa_front)*sin(x_k2(6))+ac_struct.Sa_front); %the surface area can vary depending on the angle of attack
  FD_fric = q_inf*Cf_Sa; %friction drag force
  FD_press = q_inf*interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k2(6),spd)*Sa_actual; %pressure drag force
  FD_induced = q_inf*Cd_induced*ac_struct.Sa;

  q_inf = 0.5*rho*spd*spd; %dynamic pressure
  FL = q_inf*Cl*ac_struct.Sa; %magnitude of lift force in newtons
  FD = FD_fric+FD_press+FD_induced; %q_inf*Cd*ac_struct.Sa; %magnitude of drag force in newtons

  phi = x_k2(4)-x_k2(6); %the velocity vector angle

  Lx = -FL*sin(phi);
  Ly = FL*cos(phi);

  Dx = -FD*cos(phi);
  Dy = -FD*sin(phi);

  %calculate the acting moment given the inherent vehicle moment and the elevator effect
  M = q_inf*Cm*ac_struct.Sa*ac_struct.c; %moment on craft in N-m

  %knowing the forces and moments calculate the accelerations
  ax = (Dx+Lx)/ac_struct.m;
  ay = ((Dy+Ly)/ac_struct.m)-9.81;
  ang_vel_damp = w_damp*x_k2(5)*x_k2(5);
  if x_k2(5)>0
    ang_vel_damp = -ang_vel_damp;
  endif
  ang_accel = (M)/ac_struct.MOI_y+ang_vel_damp;

  %k1 vector
  k3 = [x(2);
        ay;
        ax;
        x(5);
        ang_accel];

  x_k3 = [(x(1)+dt*k3(1));
          (x(2)+dt*k3(2));
          (x(3)+dt*k3(3));
          (x(4)+dt*k3(4));
          (x(5)+dt*k3(5));
          0];

  %AoA_k3 = x_k3(4)-atan2(x_k3(2),x_k3(3));
  O = [cos(x_k3(4)),sin(x_k3(4))];
  V = [x_k3(3),x_k3(2)];

  x_k3(6) = calculateAOA(O,V); %AoA_k3;

  %% RK4 method k4

  spd = sqrt(x_k3(2)^2+x_k3(3)^2);
  alpha_ind = findClosest1D(LT_alpha_ref,x_k3(6));
  vel_ind = findClosest1D(LT_vel_ref,spd);

  Cl = interpolateCoefficient(ac_struct,ac_struct.Cl,alpha_ind,vel_ind,x_k3(6),spd);
  [cf_lam,cf_turb,Cf_Sa] = coeff_friction(spd,ac_struct.c,rho,mu,ac_struct.Sa);

  Cd = cf_lam+cf_turb+interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k3(6),spd);
  Cd_induced = Cl*Cl/(pi*ac_struct.AR*ac_struct.oe);
  Cm = ac_struct.dCmda(alpha_ind,vel_ind)*x_k3(6)+ac_struct.Cm0(alpha_ind,vel_ind);

  %calculate the forces of flight given the coefficients and elevator angle
  q_inf = 0.5*rho*spd*spd; %dynamic pressure

  Sa_actual = abs((ac_struct.Sa_top-ac_struct.Sa_front)*sin(x_k3(6))+ac_struct.Sa_front); %the surface area can vary depending on the angle of attack
  FD_fric = q_inf*Cf_Sa; %friction drag force
  FD_press = q_inf*interpolateCoefficient(ac_struct,ac_struct.Cd,alpha_ind,vel_ind,x_k3(6),spd)*Sa_actual; %pressure drag force
  FD_induced = q_inf*Cd_induced*ac_struct.Sa;

  q_inf = 0.5*rho*spd*spd; %dynamic pressure
  FL = q_inf*Cl*ac_struct.Sa; %magnitude of lift force in newtons
  FD = FD_fric+FD_press+FD_induced; %q_inf*Cd*ac_struct.Sa; %magnitude of drag force in newtons

  phi = x_k3(4)-x_k3(6); %the velocity vector angle

  Lx = -FL*sin(phi);
  Ly = FL*cos(phi);

  Dx = -FD*cos(phi);
  Dy = -FD*sin(phi);

  %calculate the acting moment given the inherent vehicle moment and the elevator effect
  M = q_inf*Cm*ac_struct.Sa*ac_struct.c; %moment on craft in N-m

  %knowing the forces and moments calculate the accelerations
  ax = (Dx+Lx)/ac_struct.m;
  ay = ((Dy+Ly)/ac_struct.m)-9.81;
  ang_vel_damp = w_damp*x_k3(5)*x_k3(5);
  if x_k3(5)>0
    ang_vel_damp = -ang_vel_damp;
  endif
  ang_accel = (M)/ac_struct.MOI_y+ang_vel_damp;

  %k1 vector
  k4 = [x(2);
        ay;
        ax;
        x(5);
        ang_accel];

  %=============================================================================

  %using the RK4 coefficients determine the state at the next instance
  x(1) = h(ii)  +(k1(1)+k2(1)+k2(1)+k3(1)+k3(1)+k4(1))*dt/6;
  x(2) = v(2,ii)+(k1(2)+k2(2)+k2(2)+k3(2)+k3(2)+k4(2))*dt/6;
  x(3) = v(1,ii)+(k1(3)+k2(3)+k2(3)+k3(3)+k3(3)+k4(3))*dt/6;
  x(4) = th(ii) +(k1(4)+k2(4)+k2(4)+k3(4)+k3(4)+k4(4))*dt/6;
  x(5) = w(ii)  +(k1(5)+k2(5)+k2(5)+k3(5)+k3(5)+k4(5))*dt/6;

  phi_new = atan2(x(2),x(3));
  O = [cos(x(4)),sin(x(4))];
  V = [x(3),x(2)];

  x(6) = calculateAOA(O,V);

  %update the history arrays
  h(ii+1) = x(1);
  v(1,ii+1) = x(3);
  v(2,ii+1) = x(2);
  th(ii+1) = x(4);
  w(ii+1) = x(5);
  aoa(ii+1) = x(6);
  s(ii+1) = s(ii)+v(1,ii)*dt;

endfor

figure()
plot(s,h)

##    f0_vy = x(2); dvydh = 0; dvydvy = 1; dvydvx = 0; dvydth = 0; dvydw = 0; dvyda = 0;
##
##    [trash,f0_ay,daydh,daydvy,daydvx,daydth,daydw,dayda] = linearize_ay(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##    [trash,f0_ax,daxdh,daxdvy,daxdvx,daxdth,daxdw,daxda] = linearize_ax(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##
##    f0_w = x(5); dwdh = 0; dwdvy = 0; dwdvx = 0; dwdth = 0; dwdw = 1; dwda = 0;
##
##    [trash,f0_alpha,dalphadh,dalphadvy,dalphadvx,dalphadth,dalphadw,dalphada] = linearize_alpha(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##    [trash,f0_daoa,ddaoadh,ddaoadvy,ddaoadvx,ddaoadth,ddaoadw,ddaoada] = linearize_aoa_dot(ac_struct,alpha_ind,vel_ind,x(1),x(2),x(3),x(4),x(5),x(6),rho,elev);
##
##    A = [dvydh,    dvydvy,    dvydvx,    dvydth,    dvydw,    dvyda;
##         daydh,    daydvy,    daydvx,    daydth,    daydw,    dayda;
##         daxdh,    daxdvy,    daxdvx,    daxdth,    daxdw,    daxda;
##         dwdh,     dwdvy,     dwdvx,     dwdth,     dwdw,     dwda;
##         dalphadh, dalphadvy, dalphadvx, dalphadth, dalphadw, dalphada;
##         ddaoadh,  ddaoadvy,  ddaoadvx,  ddaoadth,  ddaoadw,  ddaoada]; %state matrix
##
##    B = [0,f0_vy;
##         0,f0_ay;
##         0,f0_ax;
##         0,f0_w;
##         0,f0_alpha;
##         0,f0_daoa]; %input matrix (elevator and bias as columns)
