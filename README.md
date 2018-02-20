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


%DEBUGGING SECTION (NOT FOR ASSIGNMENT EVALUATION)
%{
MAPPING X PARAMETERS - USED FOR RUNNING CODE LINE BY LINE BETWEEN
FUNCTIONS
nsec = length(x)-1;
pos_int = x(1);
posdot_int = xdot_int;
posddot_int = xddot_int;
pos_fin = x(end);
posdot_fin = xdot_fin;
posdot_fin = xddot_fin;
posddot_fin= xddot_fin;
path_desired = pathx_desired;
%}


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
