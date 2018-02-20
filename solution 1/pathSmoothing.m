function [XT YT V A T]=pathSmoothing(x,y,weight,vmax,amax,ts)


xdot_int = 0.0;
ydot_int = 0.0;
xddot_int = 0.0;
yddot_int = 0.0;
xdot_fin = 0.0;
ydot_fin = 0.0;
xddot_fin = 0.0;
yddot_fin = 0.0;
t=linspace(0,1,ts)';


v_=sqrt((diff(x)/t(end)).^2+(diff(y)/t(end)).^2);
for i=1:length(x)-1
    theta_(i)=atan2(y(i+1)-y(i),x(i+1)-x(i));
end


for i=1:length(x)-1
    xt_(i,:)=x(i)+v_(i)*cos(theta_(i))*t;
    yt_(i,:)=y(i)+v_(i)*sin(theta_(i))*t;
end
tmpx=xt_';
pathx_desired=(tmpx(:));
tmpy=yt_';
pathy_desired=(tmpy(:));



[P Pdot Pddot]=pol_matrix_comp(t);

num=length(t);

[XT, xtdot_traj, xtddot_traj] = cubicspline_smoothing_optimization2(length(x)-1,x(1), xdot_int, xddot_int, x(end), xdot_fin, xddot_fin, P, Pdot, Pddot, pathx_desired, weight );


[YT, ytdot_traj, ytddot_traj] = cubicspline_smoothing_optimization2(length(y)-1,y(1), ydot_int, yddot_int, y(end), ydot_fin, yddot_fin, P, Pdot, Pddot, pathy_desired, weight );



total_velocity = sqrt(xtdot_traj.^2+ytdot_traj.^2);
total_acceleration =  sqrt(xtddot_traj.^2+ytddot_traj.^2);
I=find(total_velocity>0);
scale_vel = min(vmax./total_velocity(I));
I=find(total_acceleration>0);
scale_acc = sqrt(min(amax./total_acceleration));
scale = min(scale_vel, scale_acc);

A=sqrt(xtddot_traj.^2+ytddot_traj.^2)*scale^2;
V=sqrt(xtdot_traj.^2+ytdot_traj.^2)*scale;

T=t;
for i=1:length(x)-2
    T=[T;t(end)*i+t];
end


