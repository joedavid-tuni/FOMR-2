
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
pause(0.1);
tftree.AvailableFrames;

% set command publisher
pub = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(pub);

% get robot position
pos=getrobotpose(tftree);

% set first node as a current node 
curind=1;


Pos=pos;
i=1;

% if robot arrives last point in 0.2 error limit, then finish
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
        % get robots position
        pos=getrobotpose(tftree);    
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







