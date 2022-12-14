clear all
close all
clc

%scratch paper
p1 = [5,3,1];
p2 = [-5;0;2];

q = vec2quat(p1,p2);

quatRot(p1,q)
