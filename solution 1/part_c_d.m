


clear 
close all
clc
rosshutdown();

load moroparams

h=figure(1);

x= path(:,1)';
y= path(:,2)';
weight = 0.0005;
vmax = 0.5;
amax = 0.5;
ts=101;

% calculate smoothed route

[XT YT V A T]=pathSmoothing(x,y,weight,vmax,amax,ts);

% remove repeated vector
XT(1:ts:end)=[];
YT(1:ts:end)=[];
V(1:ts:end)=[];
A(1:ts:end)=[];
T(1:ts:end)=[];
dt=T(2)-T(1);
Rout=[XT YT];

Route=compute_desired_angle_and_derivs(Rout,dt);


% connect roscore
rosinit('127.0.0.1'); 

% create transform reader to read robot location
tftree = rostf;
tftree.AvailableFrames;

% set command publisher
pub = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(pub);


% create landmark scanner object
scanner=MoroLandmarkScanner;



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

% read current time in second
tmp=tftree.LastUpdateTime;
time_p=tmp.Sec+tmp.Nsec/1e9;

iter=0;
time=[];
predict=[];
groundtruth=[];
landm=[];

% set initial position
pos=xEst';

% set first node as a current rote node 
curind=1;
vx=0;vz=0;
i=0;
while norm(Route(end,1:2)-pos(1:2))>0.2
         
    % if robot arrives current point in 0.2 error limit, then select next
    while norm(Route(curind,1:2)-pos(1:2))>0.2  
               
        % calculate target velocity command
        [vx vz]=mycontroller(pos,Route(curind,:),vmax);
        
        % set velocity command
        velmsg.Linear.X=vx;
        velmsg.Angular.Z=vz;
        % send message to ros
        send(pub,velmsg);   
        
        % read landmarks if there are in the range
        [A,R] = scanner.scanForLandmarks();
        
        % read current time in second
        tmp=tftree.LastUpdateTime;
        time_c=tmp.Sec+tmp.Nsec/1e9;
        % calculate time displacement from last loop
        dt=time_c-time_p;
        time_p=time_c;


        % take given linear velocity and angular velocity in a vector
        du=[vx  vz];    

        % make copy of estimated location
        xVehicle = xEst;


        % estimate location based on giving velocity command
        xVehiclePred = tcomp(xVehicle,du,dt);

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
       
         else
            % notinng new found lets procced refreshing the values for next run
            xEst = xPred;
            PESt = PPred;            
        end
        
        pos=xEst';
        
        
        
        
        
        i=i+1;
        Pos(i,:)=pos;
        
        axes(h);
        hold on;
        plot(XT(1:10:end,1),YT(1:10:end,1),'r-');
        plot(Pos(:,1),Pos(:,2),'b-');
        hold on;
        drawnow;
     
        
    end 
    % go next node's position
    curind=curind+1;    
end



