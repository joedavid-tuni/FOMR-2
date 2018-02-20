

clear 
close all
clc
rosshutdown();

figure(1);
hold on;

% connect roscore
rosinit('127.0.0.1'); 

% create transform reader to read robot location
tftree = rostf;
tftree.AvailableFrames;
pause(0.1);

% create subscriber for command given to the robot
sub = rossubscriber('/cmd_vel_mux/input/teleop');

% create subscriber for camera params
subCP = rossubscriber('/camera/rgb/camera_info');
% read camera params
cinfo=receive(subCP,1);

% create subscriber for camera image
subC = rossubscriber('/camera/rgb/image_raw');
% read one frame
dimage=receive(subC,1);
img = readImage(dimage);



% Define known location of Landmarks
LandFeatures(:,1)=[5 5]';
LandFeatures(:,2)=[1 2.5]';
LandFeatures(:,3)=[-5.5 0]';


% set initial covariance estimation
PEst = diag([0.1 0.1 0.001]);

% Standart deviation errors for  control signal
UEst = diag([0.01,0.01,1.5*pi/180]).^2;

% Standard deviation errors for observation
REst = diag([2, 10*pi/180]).^2;


% get robot's initial position
xVehicleTrue=getrobotpose(tftree)';

% set initial prediction to ground truth position
xEst =xVehicleTrue;
xOdom=xVehicleTrue;

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
    
    % read next frame
    dimage=receive(subC,1);
    img = readImage(dimage);
    
    % read landmarks if they are in the range
    [A,R] = myscanForLandmarks(img,cinfo);
    
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
    % odom estimation uses just velocaity command
    xOdom=tcomp(xOdom,du,dt);
    
    
    % calculate displacement on predictions.
    u=xVehiclePred-xVehicle;
    u(3)=AngleWrapping(u(3)) ;
    
    
    % prediction step
    PPred = J1(xVehicle,u)* PEst *J1(xVehicle,u)' + J2(xVehicle,u)* UEst * J2(xVehicle,u)';    
    
    xPred = xVehiclePred; 
    
    % take so called detected landmarks
    Z=[R';A'];
    % find if someone in the range
    iFeature=find(~isnan(R));
    % take the landmarks andle and distance which is in the range
    z=Z(:,iFeature); 

        
    % if there is any landmark
    if(~isempty(z)) 
        % find the index of detected landmark's know position                    
        FeatureIndex=iFeature(1);
        xFeature=LandFeatures(:,FeatureIndex);
        % predict the observation
        zPred = DoObservationModel(xVehicle,xFeature);
            
        % get observation Jacobians
        [jH,jHxf] = GetObsJacs(xVehicle,xFeature);            
                    
        %Correction Step
        Innov = z-zPred;
        % angles have always to be checked only betwewn 0 an 2Pi
        Innov(2) = AngleWrapping(Innov(2));
            
        S = jH*PPred*jH'+REst;
        K = PPred*jH'*inv(S); 
        xEst = xPred+ K*Innov;
        
        PEst=(eye(3)-K*jH)*PPred;
      
        landm(iter)=1;
     else
        % notinng new found lets procced refreshing the values for next run
        xEst = xPred;
        PESt = PPred;
        landm(iter)=0;
     end
     time(iter)=time_p;
     groundtruth(iter,:)=pos';
     predict(iter,:)=xEst';  
     odom(iter,:)=xOdom';
     
     plot(predict(end,1),predict(end,2),'b.');
     plot(groundtruth(end,1),groundtruth(end,2),'r.');
     plot(odom(end,1),odom(end,2),'g.');
 
end


