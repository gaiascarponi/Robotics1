function[R]=element_Rot(axis,angle)
% this function takes as inpunt:
% axis = the axis around which we want to write our transformation matrix
%       (possible choices are: x, y or z)
% s = the angle that characterize this rotation this input can either be a 
%       numerical value or a sym variable (e.g. alpha, theta...)
% and returns as output the corresponding rotational matrix

% never mind, kust to make easier this script to write
s=angle;

switch axis
    case "x"
        R = [1       0        0;
             0    cos(s)   -sin(s);
             0    sin(s)   cos(s)];
    case "y"
        R = [cos(s)   0   sin(s);
                0	    1     0;
            -sin(s)     0   cos(s)];
    case "z"
        R = [cos(s)  -sin(s)    0;
             sin(s)   cos(s)    0;
               0        0       1];
    otherwise
        error('the only possible axes are x, y, z')
end