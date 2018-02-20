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