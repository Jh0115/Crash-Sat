%linearzize the equation for angular accel around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,f0,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_alpha(ac_struct,alpha_ref,vel_ref,h,vy,vx,th,w,a,rho,elev)
  dfdh = 0;
  dfdw = 0;
  dfdth = 0;
  dfdvy = 0;

  Sa = ac_struct.Sa;
  c = ac_struct.c;
  I = ac_struct.MOI_y;
  dCmda = ac_struct.dCmda(alpha_ref,vel_ref);
  Cm0 = ac_struct.Cm0(alpha_ref,vel_ref);

  dfdvx = rho*vx*Sa*c/I*(dCmda*a+Cm0);
  dfda = rho*vx*vx*Sa*c*dCmda/2/I;

  f0 = rho*vx*vx*Sa*c/2/I*(dCmda*a+Cm0);

  f = @(h,vy,vx,th,w,a) f0+dfdh*h+dfdv*v+dfdth*th+dfdw*w+dfda*a;

endfunction
