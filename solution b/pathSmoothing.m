function [XT YT V A T]=pathSmoothing(x,y,weight,vmax,amax,ts)

% x and y components of initial velocity
xdot_int = 0.0;
ydot_int = 0.0;

% x and y components of initial acceleration
xddot_int = 0.0;
yddot_int = 0.0;

% x and y components of final velocity
xdot_fin = 0.0;
ydot_fin = 0.0;

% x and y components of final velocity
xddot_fin = 0.0;
yddot_fin = 0.0;

% defininging an arbitrary time interval for each segment,  
% in this case [0, 1] and then discretizing into grid with 101 points
t=linspace(0,1,ts)';

% Computing a straight line trajectory through the grid points. We first compute 
% the velocity and the slope of the line and then define the straight line trajectory 
% with respect to these points
v_=sqrt((diff(x)/t(end)).^2+(diff(y)/t(end)).^2);

% angle of each of the segments
for i=1:length(x)-1
    theta_(i)=atan2(y(i+1)-y(i),x(i+1)-x(i));
end

% straight line trajectories between waypoints
for i=1:length(x)-1
    xt_(i,:)=x(i)+v_(i)*cos(theta_(i))*t;
    yt_(i,:)=y(i)+v_(i)*sin(theta_(i))*t;
end

% stacking the straight line trajectory into a single column vector
tmpx=xt_';
pathx_desired=(tmpx(:));
tmpy=yt_';
pathy_desired=(tmpy(:));


% This function computes the polynomial matrix P described in the slides
[P Pdot Pddot]=pol_matrix_comp(t);

num=length(t);

[XT, xtdot_traj, xtddot_traj] = cubicspline_smoothing_optimization2(length(x)-1,x(1), xdot_int, xddot_int, x(end), xdot_fin, xddot_fin, P, Pdot, Pddot, pathx_desired, weight );


[YT, ytdot_traj, ytddot_traj] = cubicspline_smoothing_optimization2(length(y)-1,y(1), ydot_int, yddot_int, y(end), ydot_fin, yddot_fin, P, Pdot, Pddot, pathy_desired, weight );



% Computing acceleration and velocity
total_velocity = sqrt(xtdot_traj.^2+ytdot_traj.^2);
total_acceleration =  sqrt(xtddot_traj.^2+ytddot_traj.^2);

I=find(total_velocity>0);

% velocity component for computing scale
scale_vel = min(vmax./total_velocity(I));

I=find(total_acceleration>0);

% acceleration component for computing scale
scale_acc = sqrt(min(amax./total_acceleration));

%SCALE FACTOR
scale = min(scale_vel, scale_acc);

A=sqrt(xtddot_traj.^2+ytddot_traj.^2)*scale^2;
V=sqrt(xtdot_traj.^2+ytdot_traj.^2)*scale;

T=t;
for i=1:length(x)-2
    T=[T;t(end)*i+t];
end


