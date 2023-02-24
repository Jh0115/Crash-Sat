%linearzize the equation for y acceleration around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,b,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_ay(ac_struct,alpha_ref,vel_ref,h,vy,vx,th,w,a,rho,elev)
  dfdh = 0;
  dfdw = 0;
  dfdvy = 0;

  Sa = ac_struct.Sa;
  m = ac_struct.m;
  dClda = ac_struct.dClda(alpha_ref,vel_ref);
  dCdda = ac_struct.dCdda(alpha_ref,vel_ref);
  Cl0 = ac_struct.Cl0(alpha_ref,vel_ref);
  Cd0 = ac_struct.Cd0(alpha_ref,vel_ref);

  dfdvx = (Sa*rho*vx/m)*((dClda*a+Cl0)*cos(th-a)-(dCdda*a+Cd0)*sin(th-a));
  dfdth = (Sa*rho*vx*vx/2/m)*(-(dClda*a+Cl0)*sin(th-a)-(dCdda*a+Cd0)*cos(th-a));
  dfda = -(Sa*rho*vx*vx/2/m)*((dClda*a+Cl0-dCdda)*sin(a-th)+(-dCdda*a-dClda-Cd0)*cos(a-th));

  f0 = (Sa*rho*vx*vx/2/m)*((dClda*a+Cl0)*cos(th-a)-(dCdda*a+Cd0)*sin(th-a))-9.81;

  b = f0-((dfdh*h)+(dfdvy*vy)+(dfdvx*vx)+(dfdth*th)+(dfdw*w)+(dfda*a));

  f = @(h,vy,vx,th,w,a) b+dfdh.*h+dfdvy.*vy+dfdvx.*vx+dfdth.*th+dfdw.*w+dfda.*a;

endfunction

