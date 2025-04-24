%PLANAR 2R; 
    syms l1 l2 q1 q2 real;
    x = l1 * cos(q1) + l2 * cos(q1 + q2);
    y = l1 * sin(q1) + l2 * sin(q1 + q2);
    p = [x; y];

%PLANAR 3R;
 syms q1 q2 q3 L1 L2 L3 real;
 x = L1 * cos(q1) + L2 * cos(q1 + q2) + L3 * cos(q1 + q2 + q3);
 y = L1 * sin(q1) + L2 * sin(q1 + q2) + L3 * sin(q1 + q2 + q3);
 z = 0;  % Robot planare, z è costante
 p = [x; y; z];

%SPATIAL 3R;
syms q1 q2 q3 l real;
% Posizione dell'end-effector (px, py)
px = l * (cos(q1) + cos(q1 + q2) + cos(q1 + q2 + q3));
py = l * (sin(q1) + sin(q1 + q2) + sin(q1 + q2 + q3));

% Orientamento phi (somma degli angoli dei giunti)
phi = q1 + q2 + q3;

% Vettore r = [px; py; phi]
r = [px; py; phi];