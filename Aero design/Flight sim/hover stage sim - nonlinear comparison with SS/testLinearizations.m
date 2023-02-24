%validate derivative functions from nonlinear functions
clear all
close all
clc

ac_struct = initializeDesign();
LT_alpha_ref = ac_struct.alpha; %reference for alpha values of lookup tables
LT_vel_ref = ac_struct.vel; %reference for vel values of lookup tables

rho = 1;

%generate an arbitrary curve with the nonlinear function then draw the linearization
h = 0;
vy = 0;
vx = 25;
th = deg2rad(50);
w = 2;

O = [cos(th),sin(th)];
V = [vx,vy];

a = calculateAOA(O,V);

spd = sqrt(vx^2+vy^2);
alpha_ind = findClosest1D(LT_alpha_ref,a);
vel_ind = findClosest1D(LT_vel_ref,spd);


[f,b,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_ay(ac_struct,alpha_ind,vel_ind,h,vy,vx,th,w,a,rho,0);

%pick a variable to vary for the plot
Sa = ac_struct.Sa;
c = ac_struct.c;
I = ac_struct.MOI_y;
dCmda = ac_struct.dCmda(alpha_ind,vel_ind);
Cm0 = ac_struct.Cm0(alpha_ind,vel_ind);
m = ac_struct.m;
dClda = ac_struct.dClda(alpha_ind,vel_ind);
dCdda = ac_struct.dCdda(alpha_ind,vel_ind);
Cl0 = ac_struct.Cl0(alpha_ind,vel_ind);
Cd0 = ac_struct.Cd0(alpha_ind,vel_ind);

vx = 0.1:0.01:30;
f_lin = f(h,vy,vx,th,w,a);
f = (Sa*rho*vx.*vx/2/m).*((dClda.*a+Cl0).*cos(th-a)-(dCdda*a+Cd0).*sin(th-a));

if numel(f)==1
  f = f*ones(1,numel(vy));
  f_lin = f_lin*ones(1,numel(vy));
endif

plot(vx,f)
hold on
plot(vx,f_lin)














