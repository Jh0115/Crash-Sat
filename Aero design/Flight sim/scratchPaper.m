clear all
close all
clc

f = @(x,y) x.*y;

x = -3:0.5:3;
y = x;

[xx,yy] = meshgrid(x,y)
zz = f(xx,yy)

surf(xx,yy,zz)

