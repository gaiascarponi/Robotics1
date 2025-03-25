function[R]=RPY_Rot(axes,angles)
% this functions gives you the rotation matrix for a sequence of 3
% elemental rotations made on FIXED AXIS (here called Roll-Pitch-Yaw)
% subs(T_PHI,[beta gamma],[0 pi/2])
%RPY(xzy, alpha beta gamma)=Ay(gamma)*Az(beta)*Ax(gamma)



% just need to remember MATLAB what x y z are (I think he is autistic)
syms x y z

% creating S and s just for hobby (shorter name variables!)
S=axes;
s=angles;

% control input: you can't do the same rotation in sequence (XXZ, YYZ, XZZ
% are forbidden, for example)
for i=1:2
    if S(i)==S(i+1)
        error('you can''t do the same rotation in sequence')
    end
end

% creating the 3 matrices for the rotation
for i=1:3
    switch S(i)
        case "x"
            R{i}=element_Rot(sym(x),s(i));
        case "y"
            R{i}=element_Rot(sym(y),s(i));
        case "z"
            R{i}=element_Rot(sym(z),s(i));
        otherwise
            error('axis must be choosen between ''x'',''y'' or ''z''')
    end
end

% the resulting matrix is obtained by pre-moltiplicating each one.
% the order of definition R_x1x2x3 does NOT follow the order of the products
% R_x3(angle3) * R_x2(angle2) * R_x1(angle1) [the order is opposite now]

R=R{3}*R{2}*R{1};