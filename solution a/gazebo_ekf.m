
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


