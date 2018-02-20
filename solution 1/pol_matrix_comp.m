function [P Pdot Pddot]=pol_matrix_comp(t)

% pol_matrix_comp(t)
delt=abs(t(2)-t(1));
num=length(t);
Ad=[1 delt 0.5*delt^2;0 1 delt;0 0 1];
Bd=[1/6*delt^3;0.5*delt^2;delt];
P=zeros(num-1);
Pdot=zeros(num-1);
Pddot=zeros(num-1);
Pint=zeros(num,3);
Pdotint=zeros(num,3);
Pddotint=zeros(num,3);
for i=1:num-1
    for j=1:i-1
        temp= (Ad^(i-j))*Bd;
        P(i,j)=temp(1);
        Pdot(i,j)=temp(2);
        Pddot(i,j)=temp(3);
    end
end

for i=1:num
    temp=Ad^(i-1);
    Pint(i,:)=temp(1,:);
    Pdotint(i,:)=temp(2,:);
    Pddotint(i,:)=temp(3,:);
end

P=[zeros(1,size(P,2));P];
Pdot=[zeros(1,size(Pdot,2));Pdot];
Pddot=[zeros(1,size(Pddot,2));Pddot];

P=[Pint P];
Pdot=[Pdotint Pdot];
Pddot=[Pddotint Pddot];