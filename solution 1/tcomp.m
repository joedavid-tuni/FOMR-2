function nr=tcomp(r,u,dt)

u=dt*u;

result = r(3)+u(2);

if result > pi | result <= -pi
   result = AngleWrapping(result) ;
end

midangle= AngleWrapping((result+r(3))/2) ;
s = sin(midangle);
c = cos(midangle);


nr = [r(1)+u(1)*c; r(2)+u(1)*s;result];