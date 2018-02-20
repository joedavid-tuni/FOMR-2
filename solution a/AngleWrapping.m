%% Function to handle the transformation of angle
function angle = AngleWrapping(angle)
% to make sure angle lies in the range [-pi,pi]
if(angle>pi)
    angle=angle-2*pi;
elseif(angle<-pi)
    angle = angle+2*pi;
end;