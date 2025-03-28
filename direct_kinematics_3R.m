
syms L q1 q2 q3 real;

    % Calcolo della posizione
    px = L * cos(q1) + L * cos(q1 + q2) + L * cos(q1 + q2 + q3);
    py = L * sin(q1) + L * sin(q1 + q2) + L * sin(q1 + q2 + q3);

    p = [px; py];


 J=jacobian(p, [q1 q2 q3])