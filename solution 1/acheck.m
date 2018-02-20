function ang=acheck(ang1,ang2)
% absolute diff ang1-ang2
% ang1 [-pi p1], ang2 [-pi pi]
a=ang1-ang2;
ang=atan2(sin(a),cos(a));