%linearzize the equation for angular accel around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,dfdh,dfdv,dfdth,dfdw,dfda] = linearize_alpha(ac_struct,h,v,th,w,rho,elev)
  dfdh = 0;
  dfdw = 0;
  dfdth = 0;

  Sa = ac_struct.Sa_top
  c = ac_struct.chord_mean
  I = ac_struct.MOI_y
  dCmda = ac_struct.dCmda
  Cm0 = ac_struct.Cm0

  dfdv = rho*v*Sa*c/I*(dCmda*a+Cm0);
  dfda = rho*v*v*Sa*c*dCmda/2/I;

  f0 = rho*v*v*Sa*c/2/I*(dCmda*a+Cm0);

  f = f0+dfdh*h+dfdv*v+dfdth*th+dfdw*w+dfda*a;

endfunction
