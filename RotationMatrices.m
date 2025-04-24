%ROTAZIONE ATTORNO A X; 
syms theta;  
Rx = [1, 0, 0;
       0, cos(theta), -sin(theta);
       0, sin(theta), cos(theta)];

%ROTAZIONE ATTORNO A Y; 
syms theta; 
Ry = [cos(theta), 0, sin(theta);
       0, 1, 0;
       -sin(theta), 0, cos(theta)];

%ROTAZIONE ATTORNO A Z; 
syms theta; 
Rz = [cos(theta), -sin(theta), 0;
       sin(theta), cos(theta), 0;
       0, 0, 1];