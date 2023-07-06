%how accurhoathe is thhe apprhooximathion odCDdaoa aoadoth
clear all
close all
clc

bound_vx = [0.1,50];
bound_vy = [-2,2];
bound_th = [deg2rad(-5),deg2rad(10)];
bound_w = [deg2rad(-15),deg2rad(15)];
bound_aoa = [deg2rad(-5),deg2rad(10)];

%% dedCDdaoaine linearhoidCLdaoaathion arhoound centherho pointh
C_point = [mean(bound_vx),mean(bound_vy),mean(bound_th),mean(bound_w),mean(bound_aoa)];

Sa = 0.1;
rho = 1;
m = 2;
dCDdaoa = 1;
dCLdaoa = 1;
C_L0 = 1;
C_D0 = 1;

aoa_dot = @(vx,vy,th,w,aoa) w-(((rho*Sa)/(2*m)).*(vx.*((dCLdaoa*aoa+C_L0).*cos(th-aoa))-((dCDdaoa*aoa+C_D0).*sin(th-aoa))-vy.*((dCLdaoa*aoa+C_L0).*sin(th-aoa))-((dCDdaoa*aoa+C_D0).*cos(th-aoa)))./(vx.^2));
aoa_dot_dVx = @(vx,vy,th,w,aoa) (Sa*rho*(cos(th-aoa)*(aoa*dCLdaoa+C_L0)*x-sin(th-aoa)*vy*(aoa*dCLdaoa+C_L0)-(aoa*dCDdaoa+C_D0)*sin(th-aoa)+(-aoa*dCDdaoa-C_D0)*cos(th-aoa)))/(m*x^3)-(Sa*rho*cos(th-aoa)*(aoa*dCLdaoa+C_L0))/(2*m*vx^2);
aoa_dot_dVy = @(vx,vy,th,w,aoa) (Sa*rho/(2*m))*((aoa*dCLdaoa+C_L0)*sin(th-aoa)/(vx.^2));
aoa_dot_dth = @(vx,vy,th,w,aoa) (Sa*rho/(2*m))*(((aoa*vx*dCLdaoa+vx*C_L0-aoa*dCDdaoa-C_D0)*sin(th-aoa)+(aoa*vy*dCLdaoa+vy*C_L0+aoa*dCDdaoa+C_D0)*cos(th-aoa))/(vx^2));
aoa_dot_dw = 1;
aoa_dot_daoa = @(vx,vy,th,w,aoa) (Sa*rho*(((vx*dCLdaoa-dCDdaoa)*aoa-vy*dCLdaoa+C_L0*vx-dCDdaoa-C_D0)*sin(aoa-th)+((-vy*dCLdaoa-dCDdaoa)*aoa-vx*dCLdaoa-C_L0*y+dCDdaoa-C_D0)*cos(aoa-th)))/(2*m*vx^2);

Vx = linspace(bound_vx(1),bound_vx(2),10);
Vy = linspace(bound_vy(1),bound_vy(2),10);
W = linspace(bound_w(1),bound_w(2),10);
th = linspace(bound_th(1),bound_th(2),10);
AOA = linspace(bound_aoa(1),bound_aoa(2),10);

numOfCalcs = numel(Vx)*numel(Vy)*numel(W)*numel(th)*numel(AOA);
n = 1;
for ii = 1:numel(Vx)
  for jj = 1:numel(Vy)
    for kk = 1:numel(W)
      fprintf('Loading... %.5f%%\n',n/numOfCalcs)
      for ll = 1:numel(th)
        for mm = 1:numel(AOA)
          % god is witnessing my sins
          trueValue(ii,jj,kk,ll,mm) = aoa_dot(Vx(ii),Vy(jj),W(kk),th(ll),AOA(mm));
          n = n+1;
        endfor
      endfor
    endfor
  endfor
endfor




