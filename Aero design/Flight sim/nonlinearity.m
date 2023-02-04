%nonlinearity measurement of N-dimensional equations
clear all
close all
clc

%define inputs and equation
f = @(x,y) x.^4+y.^4+0.1;%x.*y.*y+6*x-y.*sqrt(x);
%umag2 = @(x,y) (y.*y./x./x./x/16)+(8*y.*y)+(-4*y./sqrt(x))+(1/2./x)+(4*x.*x);

p = [0,0];

rx = 1;
ry = 1;

x1 = p(1)-rx;
x2 = p(1)+rx;
y1 = p(2)-ry;
y2 = p(2)+ry;

dx = 0.05;
dy = 0.05;

%step 1, Get points in the range as described
x_arr = x1:dx:x2;
y_arr = y1:dy:y2;

%[xx,yy] = meshgrid(x_arr,y_arr);

%zz = f(xx,yy);
%dzz = umag2(xx,yy);

%surf(xx,yy,dzz)

%step 2, Do the partial derivatives vector for all points
n = 1;
for ii = 2:(numel(x_arr)-1)
  for jj = 2:(numel(y_arr)-1)
    %dxdx
    f1 = f(x_arr(ii+1),y_arr(jj));
    f2 = 2*f(x_arr(ii),y_arr(jj));
    f3 = f(x_arr(ii-1),y_arr(jj));
    dxdx = (f1-f2+f3)/(dx*dx);

    %dxdy
    f1 = f(x_arr(ii+1),y_arr(jj+1));
    f2 = -f(x_arr(ii+1),y_arr(jj-1));
    f3 = -f(x_arr(ii-1),y_arr(jj+1));
    f4 = f(x_arr(ii-1),y_arr(jj-1));
    dxdy = (f1+f2+f3+f4)/(4*dx*dy);

    %dydx
    dydx = dxdy;

    %dydy
    f1 = f(x_arr(ii),y_arr(jj+1));
    f2 = 2*f(x_arr(ii),y_arr(jj));
    f3 = f(x_arr(ii),y_arr(jj-1));
    dydy = (f1-f2+f3)/(dy*dy);

    dxdx_mat(ii-1,jj-1) = dxdx;
    dxdy_mat(ii-1,jj-1) = dxdy;
    dydx_mat(ii-1,jj-1) = dydx;
    dydy_mat(ii-1,jj-1) = dydy;

    u_mag_mat = [dxdx,dxdy;dydx,dydy];
    [V,L] = eig(u_mag_mat);

    eig_mag(jj-1,ii-1) = (max(abs(L(:,1)))).^2+(max(abs(L(:,1)))).^2;
    n = n+1;

    u_mag(jj-1,ii-1) = abs(dxdx)^2+abs(dxdy)^2+abs(dydx)^2+abs(dydy)^2;

  endfor
endfor



%u_mag_mat = [dxdx_mat,dxdy_mat;
%               dydx_mat,dydy_mat];

sum(eig_mag)

hold on
[xxx,yyy] = meshgrid(x_arr(2:end-1),y_arr(2:end-1));
surf(xxx,yyy,u_mag)
surf(xxx,yyy,(eig_mag))

for ii = 1:(numel(x_arr)-2)
  %integrate over the Y axis before the x axis
  u_mag_focus = u_mag(:,ii);
  u_mag_int(ii) = trapz(y_arr(2:end-1),u_mag_focus);

endfor

%integrate over x axis
integrand = trapz(x_arr(2:end-1),u_mag_int);

f0 =  f(p(1),p(2));
eta = integrand/f0/f0

##figure()
##dxdxf = @(x,y) y./(4*sqrt(x.^3));
##dxdyf = @(x,y) 2*y-1./(2*sqrt(x));
##dydxf = @(x,y) 2*y-1./(2*sqrt(x));
##dydyf = @(x,y) 2*x;
##
##dvdx = dydyf(xx,yy);
##
##surf(xxx,yyy,dydy_mat)
##hold on
##surf(xx,yy,dvdx)


