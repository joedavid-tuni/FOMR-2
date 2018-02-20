clear all
clc
close all
load moroparams

%storing x and y coordinates of path (from moroparams)
x= path(:,1)';
y= path(:,2)';

%weight of the function (tracking vs smoothing)
weight = 0.00005;

%maximum values of velocity and acceleration
vmax = 0.5;
amax = 0.5;

%sampling intervals
ts=101;

[XT YT V A T]=pathSmoothing(x,y,weight,vmax,amax,ts);

figure;
plot(XT,YT,'b-',x,y,'r*-')
xlabel('x position');ylabel('y position');
legend({'Trajectory','Waypoints'});

figure;
plot(T,V);
xlabel('Time');ylabel('Velocity');

figure;
plot(T,A);
xlabel('Time');ylabel('Acceleration');


