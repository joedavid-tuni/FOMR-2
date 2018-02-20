function [pos_traj, posdot_traj, posddot_traj]= cubicspline_smoothing_optimization2(nsec,pos_int, posdot_int, posddot_int, pos_fin, posdot_fin, posddot_fin, P, Pdot, Pddot, path_desired, weight )

nvar=size(P,2);
num=size(P,1)-1;
temp1=P(1,:);
temp2=P(num+1,:);
temp3=Pdot(1,:);
temp4=Pdot(num+1,:);
temp5=Pddot(1,:);
temp6=Pddot(num+1,:);




A_eq_path=[];

for i=1:nsec-1
    A_eq1_path12=[temp2 -temp1];
    A_eq2_path12=[temp4 -temp3];
    A_eq3_path12=[temp6 -temp5];
    A_eq_path12=[A_eq1_path12;A_eq2_path12;A_eq3_path12];
    A_eq_path12=[zeros(size(A_eq_path12,1),(i-1)*nvar) A_eq_path12 zeros(size(A_eq_path12,1),(nsec-i-1)*nvar)];
    A_eq_path = [A_eq_path; A_eq_path12];
end

B_eq_path=zeros((nsec-1)*3,1);

A_eq1_boundary = [temp1  zeros(1,(nsec-1)*nvar)];
A_eq2_boundary = [temp3  zeros(1,(nsec-1)*nvar)];
A_eq3_boundary = [temp5  zeros(1,(nsec-1)*nvar)];
A_eq4_boundary = [zeros(1,(nsec-1)*nvar)  temp2];
A_eq5_boundary = [zeros(1,(nsec-1)*nvar)  temp4];
A_eq6_boundary = [zeros(1,(nsec-1)*nvar)  temp6];

A_eq_boundary = [A_eq1_boundary; A_eq2_boundary; A_eq3_boundary; A_eq4_boundary; A_eq5_boundary; A_eq6_boundary ];





B_eq1_boundary = [pos_int];
B_eq2_boundary = [posdot_int];
B_eq3_boundary = [posddot_int];
B_eq4_boundary = [pos_fin];
B_eq5_boundary = [posdot_fin];
B_eq6_boundary = [posddot_fin];

B_eq_boundary = [ B_eq1_boundary; B_eq2_boundary; B_eq3_boundary; B_eq4_boundary; B_eq5_boundary; B_eq6_boundary ];

A_eq = [ A_eq_path; A_eq_boundary];
B_eq = [ B_eq_path; B_eq_boundary ];


cost_smoothness=eye(nvar*(nsec));
lincost_smoothness = zeros( nvar*(nsec), 1) ;

tracking_term=P;
blkdiagPdot=Pdot;
blkdiagPddot=Pddot;
for i=2:nsec    
    tracking_term=blkdiag(tracking_term,P);
    blkdiagPdot=blkdiag(blkdiagPdot,Pdot);
    blkdiagPddot=blkdiag(blkdiagPddot,Pddot);
end

cost_tracking = tracking_term'* tracking_term;
lincost_tracking = -tracking_term'*path_desired;
cost = weight*cost_smoothness+cost_tracking ;
lincost = weight*lincost_smoothness+lincost_tracking;

x = quadprog(cost,lincost,[],[],A_eq,B_eq);
sol2=x;
pos_traj = tracking_term*sol2;
posdot_traj=blkdiagPdot*sol2;
posddot_traj=blkdiagPddot*sol2;


