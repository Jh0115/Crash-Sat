clear all
close all
clc

%analyze effects of wind on airspeed over altitude
%first, load all data into one matrix

figure()
grid()
hold on
xlabel("Airspeed (m/s)")
ylabel("Altitude (m)")
title("Wind airspeed analysis")

for ii = 1:30
  filename = strcat("dataWindy",int2str(ii-1),".csv")
  data = csvread(filename)

  dataAlt = data(:,1)
  dataSpd = data(:,2)
  scatter(dataSpd,dataAlt,'b')
endfor

for ii = 1:15
  filename = strcat("dataControl",int2str(ii-1),".csv")
  data = csvread(filename)

  dataAlt = data(:,1)
  dataSpd = data(:,2)
  scatter(dataSpd,dataAlt,'r')
endfor
