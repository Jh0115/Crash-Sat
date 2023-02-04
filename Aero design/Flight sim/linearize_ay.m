%linearzize the equation for y acceleration around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_ay(ac_struct,h,vy,vx,th,w,a,rho,elev)
  dfdh = 0;
  dfdw = 0;
  dfdvy = 0;

  Sa = ac_struct.Sa_top;
  m = ac_struct.mass;
  dClda = ac_struct.dClda;
  dCdda = ac_struct.dCdda;
  Cl0 = ac_struct.Cl0;
  Cd0 = ac_struct.Cd0;

  dfdvx = (Sa*rho*vx/m)*((dClda*a+Cl0)*cos(th-a)-(dCdda*a+Cd0)*sin(th-a));
  dfdth = (Sa*rho*vx*vx/2/m)*(-(dClda*a+Cl0)*sin(th-a)-(dCdda*a+Cd0)*cos(th-a));
  dfda = -(Sa*rho*vx*vx/2/m)*((dClda*a+Cl0-dCdda)*sin(a-th)+(-dCdda*a-dClda-Cd0)*cos(a-th));

  f0 = (Sa*rho*vx*vx/2/m)*((dClda*a+Cl0)*cos(th-a)-(dCdda*a+Cd0)*sin(th-a));

  f = @(h,vy,vx,th,w,a) f0+dfdh*h+dfdvy*vy+dfdvx*vx+dfdth*th+dfdw*w+dfda*a;

endfunction

