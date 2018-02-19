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
