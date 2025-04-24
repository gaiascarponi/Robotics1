function[R]=axis_angle_dir(r,angle)

% just assignment to make this script easier to read (and WRITE!!)
s=angle;

% defining some useful matrices
I = eye(3);
S_r= [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];

% computing the rotational matrix
R1=r*r';
R2=(I-r*r')*cos(s);
R3=S_r*sin(s);
R=R1+R2+R3;
