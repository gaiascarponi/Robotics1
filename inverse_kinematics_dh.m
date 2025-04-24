function [q, sol_found] = inverse_kinematics_dh(DH_params, T_des, mode)
    % INPUT:
    %   DH_params = [alpha, a, d, theta] per ogni giunto (4 x 4)
    %   T_des = Matrice 4x4 desiderata (posa finale)
    %   mode = 'numeric' (default) o 'symbolic'
    % OUTPUT:
    %   q = Soluzione IK (angoli di giunto q1...q4)
    %   sol_found = true/false (se esiste soluzione)

    if nargin < 3
        mode = 'numeric';
    end

    if strcmp(mode, 'symbolic')
        syms q1 q2 q3 q4 real
        DH_params(:,4) = [q1; q2; q3; q4];
    end

    alpha = DH_params(:,1);
    a = DH_params(:,2);
    d = DH_params(:,3);
    theta = DH_params(:,4);

    %% --- FASE 1: Calcola T0_3 ---
    A1 = dh_matrix(alpha(1), a(1), d(1), theta(1));
    A2 = dh_matrix(alpha(2), a(2), d(2), theta(2));
    A3 = dh_matrix(alpha(3), a(3), d(3), theta(3));
    T0_3 = A1 * A2 * A3;

    % Posizione desiderata
    p_des = T_des(1:3, 4);

    % Approssimazione posizione polso: p_wrist = p_des - R_des*z*d4
    if strcmp(mode, 'numeric')
        d4 = d(4);
        R_des = T_des(1:3, 1:3);
        p_wrist = p_des - d4 * R_des * [0; 0; 1];
    else
        syms d4 real
        R_des = T_des(1:3, 1:3);
        p_wrist = p_des - d4 * R_des * [0; 0; 1];
    end

    %% --- FASE 2: IK per primi 3 giunti (q1, q2, q3) ---
    q1 = atan2(p_wrist(2), p_wrist(1));

    r = sqrt(p_wrist(1)^2 + p_wrist(2)^2);
    h = p_wrist(3) - d(1);

    L2 = a(2);
    L3 = sqrt(a(3)^2 + d(4)^2);  % link 3 effettivo

    D = (r^2 + h^2 - L2^2 - L3^2) / (2 * L2 * L3);

    if strcmp(mode, 'numeric') && abs(D) > 1
        warning('Posizione non raggiungibile!');
        sol_found = false;
        q = [];
        return;
    end

    q3 = atan2(sqrt(1 - D^2), D);  % elbow down

    alpha2 = atan2(h, r);
    beta = atan2(L3 * sin(q3), L2 + L3 * cos(q3));
    q2 = alpha2 - beta;

    %% --- FASE 3: q4 per orientamento parziale ---
    % Calcolo R0_3 con i primi 3 giunti
    A1 = dh_matrix(alpha(1), a(1), d(1), q1);
    A2 = dh_matrix(alpha(2), a(2), d(2), q2);
    A3 = dh_matrix(alpha(3), a(3), d(3), q3);
    T0_3 = A1 * A2 * A3;
    R0_3 = T0_3(1:3, 1:3);

    % Orientamento relativo
    R3_4 = R0_3' * R_des;

    % q4 da rotazione attorno asse z3
    q4 = atan2(R3_4(2,1), R3_4(1,1));  % rotazione intorno a z

    q = [q1; q2; q3; q4];
    sol_found = true;

    %% --- Stampa risultati ---
    if strcmp(mode, 'symbolic')
        fprintf('\n--- Soluzione IK Simbolica (DH) ---\n');
        fprintf('q1 = %s\n', char(q1));
        fprintf('q2 = %s\n', char(q2));
        fprintf('q3 = %s\n', char(q3));
        fprintf('q4 = %s\n', char(q4));
    else
        fprintf('\n--- Soluzione IK Numerica (DH) ---\n');
        fprintf('q1 = %.2f rad\n', q1);
        fprintf('q2 = %.2f rad\n', q2);
        fprintf('q3 = %.2f rad\n', q3);
        fprintf('q4 = %.2f rad\n', q4);
    end
end

function A = dh_matrix(alpha, a, d, theta)
    A = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta);
        0,           sin(alpha),             cos(alpha),             d;
        0,           0,                      0,                      1
    ];
end
