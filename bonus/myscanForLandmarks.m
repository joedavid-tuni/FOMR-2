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



