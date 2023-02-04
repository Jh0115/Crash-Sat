%linearzize the equation for angle of attack derivative around the point given
% ac_struct - structure object with inherent information of the aircraft (S_a, m, C_l, C_f, etc)
% h, v, th, w - height, velocity, theta, and omega state values in meters, m/s, radians, and rad/s

function [f,dfdh,dfdvy,dfdvx,dfdth,dfdw,dfda] = linearize_aoa_dot(ac_struct,h,vy,vx,th,w,a,rho,elev)
  dfdh = 0;
  dfdw = 1;

  Sa = ac_struct.Sa_top;
  m = ac_struct.mass;
  dClda = ac_struct.dClda;
  dCdda = ac_struct.dCdda;
  Cl0 = ac_struct.Cl0;
  Cd0 = ac_struct.Cd0;

  dfdvy = (Sa*rho/(2*m))*((aoa*dCLdaoa+C_L0)*sin(th-aoa)/(vx.^2));
  dfdvx = (Sa*rho*(cos(th-aoa)*(aoa*dCLdaoa+C_L0)*x-sin(th-aoa)*vy*(aoa*dCLdaoa+C_L0)-(aoa*dCDdaoa+C_D0)*sin(th-aoa)+(-aoa*dCDdaoa-C_D0)*cos(th-aoa)))/(m*x^3)-(Sa*rho*cos(th-aoa)*(aoa*dCLdaoa+C_L0))/(2*m*vx^2);
  dfdth = (Sa*rho/(2*m))*(((aoa*vx*dCLdaoa+vx*C_L0-aoa*dCDdaoa-C_D0)*sin(th-aoa)+(aoa*vy*dCLdaoa+vy*C_L0+aoa*dCDdaoa+C_D0)*cos(th-aoa))/(vx^2));
  dfda = (Sa*rho*(((vx*dCLdaoa-dCDdaoa)*aoa-vy*dCLdaoa+C_L0*vx-dCDdaoa-C_D0)*sin(aoa-th)+((-vy*dCLdaoa-dCDdaoa)*aoa-vx*dCLdaoa-C_L0*y+dCDdaoa-C_D0)*cos(aoa-th)))/(2*m*vx^2);

  f0 = w-(((rho*Sa)/(2*m)).*(vx.*((dCLdaoa*aoa+C_L0).*cos(th-aoa))-((dCDdaoa*aoa+C_D0).*sin(th-aoa))-vy.*((dCLdaoa*aoa+C_L0).*sin(th-aoa))-((dCDdaoa*aoa+C_D0).*cos(th-aoa)))./(vx.^2));

  f = @(h,vy,vx,th,w,a) f0+dfdh*h+dfdv*v+dfdth*th+dfdw*w+dfda*a;

end




