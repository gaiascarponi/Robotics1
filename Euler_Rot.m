function[R]=Euler_Rot(axes,angles)

% hear me out brother
% in order to use this simple function, i need you to give me these two
% simple inputs:
% axes = is a vector of sumbolic values (e.g. in your Command Window you
%        can create a vector "S" (or axes) =[x y z] where x, y, z are symbolic
%        variables, prevoiusly specified)
% angles = is another vecto/r of symbolic values (same thing, just creat a
%        vector "s" (or angles)=


% just need to remember MATLAB what x y z are (I think he is autistic)
syms x y z

% creating S and s just for hobby (shorter name variables!)
S=axes;
s=angles;
R_E=eye(3);

% input control: you can't do the same rotation in sequence (XXZ, YYZ, XZZ
% are forbidden, for example)
for i=1:length(S)-1
    if S(i)==S(i+1)
        error('you can''t do the same rotation in sequence')
    end
end

if length(S) ~= length(s)
    error('dimension of axes vector and angles vector must be equal!!')
end

% creating the 3 matrices for the rotation
for i=1:length(S)
    switch S(i)
        case {"x", "X"}
            R{i}=element_Rot(sym(x),s(i));
        case {"y", "Y"}
            R{i}=element_Rot(sym(y),s(i));
        case {"z", "Z"}
            R{i}=element_Rot(sym(z),s(i));
        otherwise
            error('axis must be choosen between ''x'',''y'' or ''z''')
    end
    R_E = R_E * R{i};
end

% the resulting matrix is obtained by post-moltiplicate each one.
% the order of definition R_x1x2x3 follows the order of the products
% R_x1(angle1) * R_x2(angle2) * R_x3(angle3)

R=R_E;
