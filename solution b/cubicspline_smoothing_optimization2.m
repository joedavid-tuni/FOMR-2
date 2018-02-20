function [pos_traj, posdot_traj, posddot_traj]= cubicspline_smoothing_optimization2(nsec,pos_int, posdot_int, posddot_int, pos_fin, posdot_fin, posddot_fin, P, Pdot, Pddot, path_desired, weight )

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

