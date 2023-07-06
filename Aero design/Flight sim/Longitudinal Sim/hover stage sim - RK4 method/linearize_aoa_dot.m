%linearzize the equation for angle of attack derivative around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,f0,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_aoa_dot(ac_struct,alpha_ref,vel_ref,h,vy,vx,th,w,a,rho,elev)
  dfdh = 0;
  dfdw = 1;

  Sa = ac_struct.Sa;
  m = ac_struct.m;
  dClda = ac_struct.dClda(alpha_ref,vel_ref);
  dCdda = ac_struct.dCdda(alpha_ref,vel_ref);
  Cl0 = ac_struct.Cl0(alpha_ref,vel_ref);
  Cd0 = ac_struct.Cd0(alpha_ref,vel_ref);

  dfdvy = (Sa*rho/(2*m))*((a*dClda+Cl0)*sin(th-a)/(vx.^2));
  dfdvx = (Sa*rho*(cos(th-a)*(a*dClda+Cl0)*vx-sin(th-a)*vy*(a*dClda+Cl0)-(a*dCdda+Cd0)*sin(th-a)+(-a*dCdda-Cd0)*cos(th-a)))/(m*vx^3)-(Sa*rho*cos(th-a)*(a*dClda+Cl0))/(2*m*vx^2);
  dfdth = (Sa*rho/(2*m))*(((a*vx*dClda+vx*Cl0-a*dCdda-Cd0)*sin(th-a)+(a*vy*dClda+vy*Cl0+a*dCdda+Cd0)*cos(th-a))/(vx^2));
  dfda = (Sa*rho*(((vx*dClda-dCdda)*a-vy*dClda+Cl0*vx-dCdda-Cd0)*sin(a-th)+((-vy*dClda-dCdda)*a-vx*dClda-Cl0*vy+dCdda-Cd0)*cos(a-th)))/(2*m*vx^2);

  f0 = w-(((rho*Sa)/(2*m)).*(vx.*((dClda*a+Cl0).*cos(th-a))-((dCdda*a+Cd0).*sin(th-a))-vy.*((dClda*a+Cl0).*sin(th-a))-((dCdda*a+Cd0).*cos(th-a)))./(vx.^2));

  f = @(h,vy,vx,th,w,a) f0+dfdh*h+dfdv*v+dfdth*th+dfdw*w+dfda*a;

end




