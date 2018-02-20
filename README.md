# Localization, Path Smoothing, Controller and Vision of Mobile Robots

# Description of the Task

1. Implement robot localization in the given map based on known feature locations using the following algorithms:
* Extended Kalman Filter
* Monte Carlo particle filter
Use the scanForLandmarks method of the provided MoroLandmarkScanner class to get measurements of the landmarks.
scanForLandmarks Return range and angle to landmarks.
* [A,R] = scanForLandmarks() returns 3 by 1 matrices R and A, where R(i)  is the range, and A(i) the angle, from robot pose to i'th landmark if  the landmark is within sensor range, and NaN otherwise.
First try using only the angle measurement, and then both the angle and range.
The landmark locations in the map coordinate frame are:
* red: [5, 5]
* green: [1, 2.5]
* blue: [-5.5, 0]
Use teleoperation to drive the robot around the environment. Gather and compare data of the robot’s actual location, odometry information, and estimated location using the two algorithms.

<a href="https://drive.google.com/uc?export=view&id=1E9QgOxiuZ_I7CKgtc7GB0dGv8PmweQ3l"><img src="https://drive.google.com/uc?export=view&id=1E9QgOxiuZ_I7CKgtc7GB0dGv8PmweQ3l" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>



2.  Implement a path smoothing algorithm

3.  Implement a path following robot controller.

4. Familiarize yourself with camera geometry and actually implement the extraction of colored balls as landmarks from the
environment. The radius of the balls is 0.15.

----------------------------------------------------------------------------------------------------------------------------------------

# Solution

## IMPLEMENTATION OF ROBOT LOCALIZATION BASED ON KNOWN FEATURE LOCATIONS USING THE EXTENDED KALMAN FILTER ALGORITHM[1][2][3]

#### Basic Algorithm[1] 
<a href="https://drive.google.com/uc?export=view&id=1KzzbBIenVKiXQ7jZa7wtIsn18ehiUhel"><img src="https://drive.google.com/uc?export=view&id=1KzzbBIenVKiXQ7jZa7wtIsn18ehiUhel" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

#### Flow Chart of the Algorithm[1]
<a href="https://drive.google.com/uc?export=view&id=1PSltEzDK-BeQaDwH-CMvJVHMoV6l60J0"><img src="https://drive.google.com/uc?export=view&id=1PSltEzDK-BeQaDwH-CMvJVHMoV6l60J0" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

The Kalman Filter Algorithm is depicted above. The Kalman Filter represents the belief bel(x t ) at time t by
the mean μ t and the covariance Σ t .
* Input: The input of the Kalman filter is the prior belief at time t − 1, represented by mean μ t−1 and covariance
Σ t−1 . To update these parameters, the Kalman filter Algorithm requires the control u t (linear and angular
velocity in the localization context of the assignment) and the measurement z t .
* Output: The output is the belief at time t, represented by mean μ t and covariance Σ t .

In Lines 2 and 3, the predicted belief μ ̄ and Σ ̄ is calculated representing the belief bel ̄ ̄ (xt) one time step
later, but before incorporating the measurement z t . This belief is obtained by incorporating the control u t .
This matrix is multiplied twice into the covariance, since the covariance is a quadratic matrix.
The belief bel ̄ ̄ (xt) is subsequently transformed into the desired belief bel(xt) in Lines 6 through 8, by
incorporating the measurement z t . The variable K t , computed in Line 6 is called Kalman gain. It specifies the
degree to which the measurement is incorporated into the new state estimate. Line 7 manipulates the
mean, by adjusting it in proportion to the Kalman gain K t and the deviation of the actual measurement,
z t , and the measurement predicted according to the measurement probability. Finally, the new covariance
of the posterior belief is calculated in Line 8, adjusting for the information gain resulting from the
measurement.



### PREDICTION

#### Detailed Prediction Step Algorithm for EKF based localization [1]
<a href="https://drive.google.com/uc?export=view&id=1H0VFyQNl6M42IWRMqUl3awUXKYpsO58a"><img src="https://drive.google.com/uc?export=view&id=1H0VFyQNl6M42IWRMqUl3awUXKYpsO58a" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

### CORRECTION STEP /MEASUREMENT UPDATE

#### Detailed Correction Step Algorithm for EKF based Localization [1]
<a href="https://drive.google.com/uc?export=view&id=1O-OB7-x-bSq9ZyBzHvONL9t-1LURau7j"><img src="https://drive.google.com/uc?export=view&id=1O-OB7-x-bSq9ZyBzHvONL9t-1LURau7j" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>


* Line 1 calculates the predicted measurement (distance and angle)
* Line 2 calculates the Jacobian of the measurement model
* Line 3 is the Observation Noise Covariance Matrix.
* Line 4 calculates the uncertainty corresponding to the observed measurement.
* Line 5 calculates the Kalman Gain
* Lines 6 and 7 is where the estimates are updated by the mean and covariance


### Source Code

_Only the Main Code is shown here. Smaller functions can be checked in the repository_

```matlab
clear 
close all
clc
rosshutdown();

h=figure(1);

% connect to the ROS Network
rosinit('127.0.0.1'); 

% creates a ROS transformation tree object to read robot location
tftree = rostf;
tftree.AvailableFrames;

% create a ROS subscriber for command given to the robot
sub = rossubscriber('/cmd_vel_mux/input/teleop');

% create a landmark scanner object
scanner=MoroLandmarkScanner;

% Define known location of Landmarks
LandFeatures(:,1)=[5 5]';
LandFeatures(:,2)=[1 2.5]';
LandFeatures(:,3)=[-5.5 0]';


% set initial covariance estimation
PEst = diag([0.1 0.1 0.001]);


% Motion Noise Covariance Matrix (Q)
UEst = diag([0.01,0.01,1.5*pi/180]).^2;

% Observation Noise Covariance Matrix (R)
ZEst = diag([2, 10*pi/180]).^2;

% get robot's initial position
xVehicleTrue=getrobotpose(tftree)';

% set initial prediction to ground truth position
xEst =xVehicleTrue;

% read current time in second
tmp=tftree.LastUpdateTime;
time_p=tmp.Sec+tmp.Nsec/1e9;

iter=0;
time=[];
predict=[];
groundtruth=[];
landm=[];

while(1)
    iter=iter+1;
    
    % get current ground truth position
    pos=getrobotpose(tftree)';
    % read landmarks if there are in the range
    [A,R] = scanner.scanForLandmarks();
    % get given velocity command from keyboard by user
    vel = receive(sub,1);
    
    % read current time in second
    tmp=tftree.LastUpdateTime;
    time_c=tmp.Sec+tmp.Nsec/1e9;
    % calculate time displacement from last loop
    dt=time_c-time_p;
    time_p=time_c;
    
    
    % take given linear velocity and angular velocity in a vector
    du=[vel.Linear.X  vel.Angular.Z];    
    
    % make copy of estimated location
    xVehicle = xEst;
    
    % estimate location based on giving velocity command
    xVehiclePred = tcomp(xVehicle,du,dt);
    
    % calculate displacement on predictions.
    u=xVehiclePred-xVehicle;
    u(3)=AngleWrapping(u(3)) ;
    
    % Predicted Covariance (sigma_bar)
    PPred = J1(xVehicle,u)* PEst *J1(xVehicle,u)' + J2(xVehicle,u)* UEst * J2(xVehicle,u)';    

    
    % Predicted mean (mu_bar)
    xPred = xVehiclePred;
    
    
    % take so called detected landmarks
    Z=[R';A'];
    % find if someone in the range
    iFeature=find(~isnan(R));
    % take the landmarks angle and distance which is in the range
    z=Z(:,iFeature); 

        
    % if there is any landmark
    if(~isempty(z)) 
        
        % find the index of detected landmark's know position                    
        FeatureIndex=iFeature(1);
        xFeature=LandFeatures(:,FeatureIndex);
        
        % predicted measurement mean
        zPred = DoObservationModel(xVehicle,xFeature);
            
        % get observation Jacobians
        [jH,jHxf] = GetObsJacs(xVehicle,xFeature);            
                    
        % Innovation
        Innov = z-zPred;
        
        % angles have always to be checked always to lie betwewn 0 an 2Pi
        Innov(2) = AngleWrapping(Innov(2));
           
        %Innovation Covariance
        S = jH*PPred*jH'+ZEst;
        
        %Kalman Gain
        K = PPred*jH'*inv(S); 
        
        %updated mean (mu)
        xEst = xPred+ K*Innov;
      
        
        %updated covariance (sigma)
        PEst=(eye(3)-K*jH)*PPred;
      
      
        landm(iter)=1;
     else
        % nothing new found lets proceed to the next iteration after refreshing the values
        xEst = xPred;
        PESt = PPred;
        landm(iter)=0;
     end
     time(iter)=time_p;
     groundtruth(iter,:)=pos';
     predict(iter,:)=xEst';      
     axes(h);
     hold on;
     plot(predict(:,1),predict(:,2),'b-');
     plot(groundtruth(:,1),groundtruth(:,2),'r-');
     hold on;
     drawnow;
 
end
```

## Path Smoothing[1]

### Literature

<a href="https://drive.google.com/uc?export=view&id=1S5D8S9c9-E1fmI9Lye-tLRxagJ9j9axe"><img src="https://drive.google.com/uc?export=view&id=1S5D8S9c9-E1fmI9Lye-tLRxagJ9j9axe" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

<a href="https://drive.google.com/uc?export=view&id=1ymEhg2tvydelJAWS4WCjTb3-4GMDqpo1"><img src="https://drive.google.com/uc?export=view&id=1ymEhg2tvydelJAWS4WCjTb3-4GMDqpo1" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

<a href="https://drive.google.com/uc?export=view&id=14pknF43iwT13yRF3-PBI9jsCzvUnTagF"><img src="https://drive.google.com/uc?export=view&id=14pknF43iwT13yRF3-PBI9jsCzvUnTagF" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

### Source Code

Main function pathSmoothing() that computes position, velocity and acceleration after calling cubicspline_smoothing_optimzation() function

```matlab
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



```

Funciton cubicspline_smoothing_optimzation() that returns position, velocity and acceleration

```matlab
function [pos_traj, posdot_traj, posddot_traj]= cubicspline_smoothing_optimization(nsec,pos_int, posdot_int, posddot_int, pos_fin, posdot_fin, posddot_fin, P, Pdot, Pddot, path_desired, weight )

nvar=size(P,2);
num=size(P,1)-1;
temp1=P(1,:);       % creating a temporary array with the first row of P matrix
temp2=P(num+1,:);   % creating a temporary array with the last row of the P matrix
temp3=Pdot(1,:);    % creating a temporary array with the first row of the Pdot matrix
temp4=Pdot(num+1,:);% creating a temporary array with the last row of the Pdot matrix
temp5=Pddot(1,:);   % creating a temporary array with the first row of the Pddot matrix
temp6=Pddot(num+1,:);% creating a temporary array with the last row of the Pddot matrix

% Below we construct the stitching constraints in the form AX=B where X is the variable
%% Constructing the A matrix of the stitching and boundary constraints

% STITCHING Constraints
A_eq_path=[];

for i=1:nsec-1
    A_eq1_path12=[temp2 -temp1];    % stitching at the position level
    A_eq2_path12=[temp4 -temp3];    % stitching at the velocity level
    A_eq3_path12=[temp6 -temp5];    % stitching at the acceleration level
    A_eq_path12=[A_eq1_path12;A_eq2_path12;A_eq3_path12]; % stacking the above stitching constraints in a single matrix
    
    % the generalizing equation makes sure that the dimension of the matrix
    % does not change (1442 in this case) by adding matrices of zeros at
    % the beginning and at the end with the multiplication indices as (i-1)
    % and (nsec-i-1) such throughout the for loop iteration such that
    % their sum is 12
    
    
    A_eq_path12=[zeros(size(A_eq_path12,1),(i-1)*nvar) A_eq_path12 zeros(size(A_eq_path12,1),(nsec-i-1)*nvar)]; % filling the irrelevant matrix part with zeroes
    A_eq_path = [A_eq_path; A_eq_path12]; % stacking all stitching constraints in one final matrix
end

% BOUNDARY Constraints

% Initial boundary Constraints
A_eq1_boundary = [temp1  zeros(1,(nsec-1)*nvar)];   % Initial point boundary constraints.  Everything but starting elements are zeros
A_eq2_boundary = [temp3  zeros(1,(nsec-1)*nvar)];   % Initial velocity constraints. Everything but starting elements are zeros
A_eq3_boundary = [temp5  zeros(1,(nsec-1)*nvar)];   % Inital acceleration boundary constraints.  Everything but starting elements are zeros.

% Final boundary Constraints
A_eq4_boundary = [zeros(1,(nsec-1)*nvar)  temp2];   % Final position constraints. Everything but ending elements are zeros.
A_eq5_boundary = [zeros(1,(nsec-1)*nvar)  temp4];   % Final velocity constraints. Everything but ending elements are zeros.
A_eq6_boundary = [zeros(1,(nsec-1)*nvar)  temp6];   % Final acceleration constraints. Everything but ending elements are zeros.

A_eq_boundary = [A_eq1_boundary; A_eq2_boundary; A_eq3_boundary; A_eq4_boundary; A_eq5_boundary; A_eq6_boundary ]; % statcking all boundary constraints in one matrix.



%% Constructing the B matrix of the stitching and boundary constraints

% STITCHING Constraints
B_eq_path=zeros((nsec-1)*3,1);

% BOUNDARY Constraints

B_eq1_boundary = [pos_int];     % initial position boundary constraints
B_eq2_boundary = [posdot_int];  % initial velocity boundary constraints
B_eq3_boundary = [posddot_int]; % initial acceleration constraints
B_eq4_boundary = [pos_fin];     % final position constraints
B_eq5_boundary = [posdot_fin];  % final velocity constraints
B_eq6_boundary = [posddot_fin]; % final acceleration constraint

B_eq_boundary = [ B_eq1_boundary; B_eq2_boundary; B_eq3_boundary; B_eq4_boundary; B_eq5_boundary; B_eq6_boundary ]; % stacking B matrix of boundary constraints in one big matrix

A_eq = [ A_eq_path; A_eq_boundary]; % stacking stitching and boundary constraints together
B_eq = [ B_eq_path; B_eq_boundary ];

%% Construction of objective function

cost_smoothness=eye(nvar*(nsec));   % Q_{smoothness} discussed in the slides
lincost_smoothness = zeros( nvar*(nsec), 1) ;  % q_{smoothness} discussed in the slides

tracking_term=P;
blkdiagPdot=Pdot;
blkdiagPddot=Pddot;
for i=2:nsec    
    tracking_term=blkdiag(tracking_term,P);
    blkdiagPdot=blkdiag(blkdiagPdot,Pdot);
    blkdiagPddot=blkdiag(blkdiagPddot,Pddot);
end

cost_tracking = tracking_term'* tracking_term;      % Q_{tracking}
lincost_tracking = -tracking_term'*path_desired;    % q_{tracking}
cost = weight*cost_smoothness+cost_tracking ;       % w*Q_{smoothness}+Q_{tracking}
lincost = weight*lincost_smoothness+lincost_tracking;   % w*q_{smoothness}+q_{tracking}


%% Calling a solver quadprog to solve the optimization problem
x = quadprog(cost,lincost,[],[],A_eq,B_eq); % solving the optimization problem
sol2=x; % extracting the solution 
pos_traj = tracking_term*sol2;  % final position trajectory
posdot_traj=blkdiagPdot*sol2;   % final velocity trajectory
posddot_traj=blkdiagPddot*sol2; % final acceleration trajectory



```

Function pol_matix_comp that computes that matrix as defined in the Literature Section
```matlab
function [P Pdot Pddot]=pol_matrix_comp(t)

% This function computes the polynomial matrix P described in the slides
delt=abs(t(2)-t(1));
num=length(t);
Ad=[1 delt 0.5*delt^2;0 1 delt;0 0 1];
Bd=[1/6*delt^3;0.5*delt^2;delt];
P=zeros(num-1);
Pdot=zeros(num-1);
Pddot=zeros(num-1);
Pint=zeros(num,3);
Pdotint=zeros(num,3);
Pddotint=zeros(num,3);
for i=1:num-1 %debug point 1
    for j=1:i-1
        temp= (Ad^(i-j))*Bd;
        P(i,j)=temp(1);
        Pdot(i,j)=temp(2);
        Pddot(i,j)=temp(3);
    end
end

for i=1:num %debug point 2
    temp=Ad^(i-1);
    Pint(i,:)=temp(1,:);
    Pdotint(i,:)=temp(2,:);
    Pddotint(i,:)=temp(3,:);
end

P=[zeros(1,size(P,2));P];
Pdot=[zeros(1,size(Pdot,2));Pdot];
Pddot=[zeros(1,size(Pddot,2));Pddot];

P=[Pint P];
Pdot=[Pdotint Pdot];
Pddot=[Pddotint Pddot];


```
### Output Graphs

#### Graph of optimized trajectory

<a href="https://drive.google.com/uc?export=view&id=18lhHQaon-NZSWVT8EJAtGjTFUgwIodCm"><img src="https://drive.google.com/uc?export=view&id=18lhHQaon-NZSWVT8EJAtGjTFUgwIodCm" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

18lhHQaon-NZSWVT8EJAtGjTFUgwIodCm

#### Graph of Velocity and Acceleration

<a href="https://drive.google.com/uc?export=view&id=1TD0vMuwdOnOdKbVJUX5QtQ3FxA9Z5xnQ"><img src="https://drive.google.com/uc?export=view&id=1TD0vMuwdOnOdKbVJUX5QtQ3FxA9Z5xnQ" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

## Robot Controller

Two solutions are implemented in this section. The first one uses the Extended Kalman Filter developed
earlier to localize the robot and uses the smoothed path generated in part b and a controller to provide a
means of control to the robot.
In the second solution, a linear (proportional) path following controller was implemented. The error
between a desired heading and actual heading is calculated and this error is in turn multiplied by a linear
coefficient, which is given as an angular velocity input to the robot. This does not use the EKF
The main scripts that the algorithm uses, besides those described earlier is described below.
The Route function computes a matrix Route that consists of all the x coordinates in the 1 st column, the y
coordinates of the 2 nd column(already passed as argument - Rout), angle theta of vector of current node
with that of the next node in the 3 rd column, derivative of x, y and theta in the 4 th , 5 th and 6 th columns
respectively.

```matlab
function Route=compute_desired_angle_and_derivs(Rout,dt)
Route=Rout;
% set desired theta with the angle of vector from current to the next node
Route(2:end,3)=atan2(Route(2:end,2)-Route(1:end-1,2),Route(2:end,1)-Route(1:end-1,1));
Route(1,3)=Route(2,3);
% set desired derivative of x
Route(2:end,4)=( Route(2:end,1)-Route(1:end-1,1))/dt;
%set desired derivative of y
Route(2:end,5)=( Route(2:end,2)-Route(1:end-1,2))/dt;
%set desired derivative of theta
Route(2:end,6)=( Route(2:end,3)-Route(1:end-1,3))/dt;
```

The mycontroller function is called repetitively. A linear velocity is arbitrarily selected and the distance
between the robots current location and the next point is calculated. Also calculates is the difference
between of the angle between the next and current nodes. Error is calculated and is input to the bot
proportionally as angular velocity by a linear coefficient.

```matlab
function [vx vz]=mycontroller(pos,goal,vmax)
% choosee arbitary Linear Velocity
vx=0.1;
% distance between robot post and target
dst=norm([pos(1:2)-goal(1:2)]);
% angular difference between target and robots actual angle
ang=acheck(atan2(goal(2)-pos(2),goal(1)-pos(1)),pos(3));
% ye error according to robot frame
ye=dst*sin(ang);
% determine angular velocitywith multiply coefficient k2 to ye
vz=5*ye;
if (vz>vmax)
vz=vmax;
end
if (vz<-vmax)
vz=-vmax;
end
```

## Gazebo Simuations of all of the above algorithms

### Localization using Extended Kalman Filter

When the turtle bot sets off in the beginning, there is clearly not much of an error in the estimation of the
posterior.

<a href="https://drive.google.com/uc?export=view&id=1nOGLaGAVGCmCKKNZLbVllAqmjAzhfg1J"><img src="https://drive.google.com/uc?export=view&id=1nOGLaGAVGCmCKKNZLbVllAqmjAzhfg1J" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

When the turtlebot turns right and moves towards the next room, the uncertainty in the robot pose
increases and this is shown as the deviation of the robot path as shown in Figure 10. However, when the
second landmark is in the vicinity of the turtle bot, it can be seen that the uncertainty decreases as shown
in the plot in figure below

<a href="https://drive.google.com/uc?export=view&id=1Y5Nh3W_TnEdn08dGysZUOxY192bNDYEp"><img src="https://drive.google.com/uc?export=view&id=1Y5Nh3W_TnEdn08dGysZUOxY192bNDYEp" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

Similar observations are found as the turtlebot leaves the second landmark and approaches the third
landmark.



<a href="https://drive.google.com/uc?export=view&id=1ZBmRjlmDJIiXXSfe5v81vEahEVNLxTcm"><img src="https://drive.google.com/uc?export=view&id=1ZBmRjlmDJIiXXSfe5v81vEahEVNLxTcm" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

###  ROBOT FOLLOWING ROBOT CONTROLLER Using smoothed Trajectory

The smoothed path obtained was shown in the previous section. Its implementation using a controller to
follow the smoothed part is shown in the next section.

#### Starting the smoothed trajectory
<a href="https://drive.google.com/uc?export=view&id=1zmwcgOHF8SKOUNd_0fgMUdVL0MOVu7UQ"><img src="https://drive.google.com/uc?export=view&id=1zmwcgOHF8SKOUNd_0fgMUdVL0MOVu7UQ" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

#### Intermediate times in the trajectory
<a href="https://drive.google.com/uc?export=view&id=1kPWcNug6yYCCPPtTpJInSqjqnwQtdRDk"><img src="https://drive.google.com/uc?export=view&id=1kPWcNug6yYCCPPtTpJInSqjqnwQtdRDk" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

#### Halfway down the smoothed trajectory
<a href="https://drive.google.com/uc?export=view&id=1ygBgK9vzyb1SPS19DOIzyhIMPwHnvZJF"><img src="https://drive.google.com/uc?export=view&id=1ygBgK9vzyb1SPS19DOIzyhIMPwHnvZJF" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

#### Turtlebot completes the smoothed trajectory
<a href="https://drive.google.com/uc?export=view&id=1WAzdZYO6q46oUznyBg__HAuCXW0uRIIj"><img src="https://drive.google.com/uc?export=view&id=1WAzdZYO6q46oUznyBg__HAuCXW0uRIIj" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

## EXTRACTION OF COLOURED BALLS AS LANDMARKS [4][5]

The aim of this section is about detecting certain coloured objects at the image and then find its real
distance and relative angle in terms of view point. To detect certain object on the image, first we analyze
the concerned objects in the simulation.
Every color in the image can be presented by 3 component as R,G and B. As our analysis the blue ball’s
pixels R band is all 0, Green band is all zero, but blue component is at about 110 intensity values. So to
detect blue ball, we design a filter which filter the pixels whose red band less than 10, green band less than
10 but blue bands is bigger than 100 intensive value. In matlab code we implement this filter with following
codes;

```matlab
% looking for blue object.
n=find(I(:,:,1)<10 & I(:,:,2)<10 & I(:,:,3)>100);
B=zeros(size(I,1),size(I,2));
B(n)=1;
```
Likewise, for the green ball, we follow the same procedure but just the differences is that the green
component should be higher than 100 , other bands should be less than 10 and similarly with the red ball.
For red ball, just red band should be higher than 100 but others should be less than 10. Green and red ball
detection are implemented with following codes;

```matlab
% looking for blue object.
n=find(I(:,:,1)<10 & I(:,:,2)<10 & I(:,:,3)>100);
B=zeros(size(I,1),size(I,2));
B(n)=1;
```

```matlab
% looking for blue object.
n=find(I(:,:,1)<10 & I(:,:,2)<10 & I(:,:,3)>100);
B=zeros(size(I,1),size(I,2));
B(n)=1;
```

The following figure shows how designed filters run for some example input images I.

<a href="https://drive.google.com/uc?export=view&id=1F9dPOUfzcd8znxeApStCbN1RWW322EIA"><img src="https://drive.google.com/uc?export=view&id=1F9dPOUfzcd8znxeApStCbN1RWW322EIA" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

After filtering given input image, we need to do connected component analysis and figure out how many different object are there to do this, we used bwlabel(B) command. For given example there is already just one white component, so it does not produce different image but it could include some other objects too on the image. Bwlabel command basically figures how many components are there. Next, we calculate every single component’s area, major axis and center. To do this we used following matlab comamnd.

```matlab

```

With this command we obtain labeled object center x and y, the maximum length of ball (diameter in this sense), and area to approve if the detected object is not too small like a noise.
We implement that procedure for all the red, green and blue balls detection. To determine which filters
results is prominent than the others, we selected the object which has maximum area. If the maximum
area is less than the certain threshold we neglect this object. But if it is enough big as being a ball, then we
calculate its distance and view-angle as Figure shown below.

<a href="https://drive.google.com/uc?export=view&id=1lWs4liITi6-wtJOyY97zso1AOfn-2iI5"><img src="https://drive.google.com/uc?export=view&id=1lWs4liITi6-wtJOyY97zso1AOfn-2iI5" style="width: 500px; max-width: 100%; height: auto" title="Click for the larger version." /></a>

To calculate how far the object is we used pinehole camera model. According to model at above figure, we can write folowing equation: 
major axis on the image = focal length of camera * real major axis(diameter)/ distance
In order to calculate the distance we used following code,

```matlab
 % major axis/ focal lenght = actual ball lenght(0.3)/ actual distance
        Rr(Regions(n,1),1)=f*0.3/Regions(n,3);
```

Where Regions(n,3) means concerned object major axis on the screen, f means focal length of camera and 0.3 means real diameter of ball
To calculate view angle of concerned object we used again pinehole camera model and this time we calculate what angle the object is according to center line. To do this we used following code;

```matlab
 % major axis/ focal lenght = actual ball lenght(0.3)/ actual distance
        Rr(Regions(n,1),1)=f*0.3/Regions(n,3);
```

In this code cx means center of image on x axis. Our images has 640x480 resolution so it has 640 pixel width,
according to given camera parameters the center of the camera view on x axis is 320.5. Regions(n,4) means
the x component of detected object’s center. So cx-Regions(n,4) means how detected object is far from the
center. And if we take the angle consist of cx-Regions(n,4) and f, we can obtain the view angle of detected
object.

#### Function myscanForLandmarks()

```matlab
function [A,Rr] = myscanForLandmarks(I,cinfo)
%this function takes image and camera params and return landmark distance 
%and relative angle for one blue green and red landmark.
A=[nan;nan;nan];
Rr=[nan;nan;nan];
Regions=[];

% looking for blue object.
n=find(I(:,:,1)<10 & I(:,:,2)<10 & I(:,:,3)>100);
B=zeros(size(I,1),size(I,2));
B(n)=1;
% find correspodings areas center major axis and total area
statsB = regionprops(bwlabel(B),'Centroid','MajorAxisLength','Area');
% Add findings to region variable
for i=1:length(statsB)
    Regions=[Regions;3 statsB(i).Area statsB(i).MajorAxisLength statsB(i).Centroid];
end



% looking for green object.
n=find(I(:,:,1)<10 & I(:,:,2)>100 & I(:,:,3)<10);
G=zeros(size(I,1),size(I,2));
G(n)=1;
% find correspodings areas center major axis and total area
statsG = regionprops(bwlabel(G),'Centroid','MajorAxisLength','Area');
for i=1:length(statsG)
    Regions=[Regions;2 statsG(i).Area statsG(i).MajorAxisLength statsG(i).Centroid];
end

% looking for red object.
n=find(I(:,:,1)>100 & I(:,:,2)<10 & I(:,:,3)<10);
R=zeros(size(I,1),size(I,2));
R(n)=1;
% find correspodings areas center major axis and total area
statsR = regionprops(bwlabel(R),'Centroid','MajorAxisLength','Area');
for i=1:length(statsR)
    Regions=[Regions;1 statsR(i).Area statsR(i).MajorAxisLength statsR(i).Centroid];
end


% if we find something
if size(Regions,1)>0    

    % find biggest object
    [m n]=max(Regions(:,2));
    
    % if the biggest object is enough big
    
    if m>200

        % find how far the biggest landmark is.
        % focal length
        f=cinfo.K(1);
        % major axis/ focal lenght = actual ball lenght(0.3)/ actual distance
        Rr(Regions(n,1),1)=f*0.3/Regions(n,3);
        % center x 
        cx=cinfo.K(3);
        
        % view angle= atan(cx-ox,focal lenght)
        A(Regions(n,1),1)=atan2(cx-Regions(n,4),f);
        
    end
end    

[A Rr]; %remove semicolon to see landmark details
n=0;
n = find(~isnan(Rr));

if n == 1
    disp('red landmark detected')
    
elseif n == 2 
    disp('green landmark detected')

elseif n == 3
    disp('blue landmark detected')
    
else
    disp('no landmarks detected')
end

```
## References

[1] Lecture Slides, Introduction to Robotics, Prof. Wolfram Burgard, Albert-Ludwigs-Universität Freiburg

[2] Lecture Slides, Fundamentals of Mobile Robotics, Prof. Reza Ghabcheloo, Tampere University of Technology

[3] “Probabilistic Robotics”, Sebastian Thrun, Wolfram Burgard, Dieter Fox

[4] Lecture 12 Slides, Computer Vision 1, Penn State College of Engineering

[5] “Track and Follow and Object”, Mathworks Documentation,
https://se.mathworks.com/help/robotics/examples/track-and-follow-an-object.html

[6] “Offline and Online Trajectory Planning”, Zvi Shiller

