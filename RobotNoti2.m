%% Inizializzazione
close all; clear; clc;
figure('Name', 'Forward Kinematics Demo', 'Position', [100 100 1200 800]);

%% Parametri dei robot
syms L1 L3 L3 q1 q2 q3 real; % Lunghezze dei bracci
                             % + Variabili di giunto [q1, q2, q3]

%% 1. Robot RRR - PLANAR (3R giunti rotoidali)
disp('=== Robot RRR ===');
p_RRR = [
    L1*cos(q(1)) + L2*cos(q(1)+q(2)) + L3*cos(q(1)+q(2)+q(3));
    L1*sin(q(1)) + L2*sin(q(1)+q(2)) + L3*sin(q(1)+q(2)+q(3));
    0
];
fprintf('Posizione RRR: [%.2f, %.2f, %.2f]\n', p_RRR);

%% 2. Robot RRR - SPATIAL (3R giunti rotoidali)
disp('=== Robot RRR ===');
s_RRR = [
    L1*cos(q(1)) + L2*cos(q(1)+q(2)) + L3*cos(q(1)+q(2)+q(3));
    L1*sin(q(1)) + L2*sin(q(1)+q(2)) + L3*sin(q(1)+q(2)+q(3));
    q1+q2+q3;
];
fprintf('Posizione RRR: [%.2f, %.2f, %.2f]\n', s_RRR);

%% 3. Robot RPR (Revolute-Prismatic-Revolute)
disp('=== Robot RPR ===');
p_RPR = [
    (L1 + q(2))*cos(q(1));
    (L1 + q(2))*sin(q(1));
    L3
];
fprintf('Posizione RPR: [%.2f, %.2f, %.2f]\n', p_RPR);

%% 3. Robot PRR (Prismatic-Revolute-Revolute)
disp('=== Robot PRR ===');
p_PRR = [
    q(1) + L2*cos(q(2)) + L3*cos(q(2)+q(3));
    L2*sin(q(2)) + L3*sin(q(2)+q(3));
    0
];
fprintf('Posizione PRR: [%.2f, %.2f, %.2f]\n', p_PRR);

%% 4. Robot SCARA (RRP)
disp('=== Robot SCARA ===');
p_SCARA = [
    L1*cos(q(1)) + L2*cos(q(1)+q(2));
    L1*sin(q(1)) + L2*sin(q(1)+q(2));
    q(3)
];
fprintf('Posizione SCARA: [%.2f, %.2f, %.2f]\n', p_SCARA);

%% 5. Robot Cilindrico (RPP)
disp('=== Robot Cilindrico ===');
p_Cylindrical = [
    q(2)*cos(q(1));
    q(2)*sin(q(1));
    q(3)
];
fprintf('Posizione Cilindrico: [%.2f, %.2f, %.2f]\n', p_Cylindrical);

%% 6. Robot Sferico (RRP)
disp('=== Robot Sferico ===');
p_Spherical = [
    q(3)*cos(q(1))*sin(q(2));
    q(3)*sin(q(1))*sin(q(2));
    q(3)*cos(q(2))
];
fprintf('Posizione Sferico: [%.2f, %.2f, %.2f]\n', p_Spherical);

%% 7. Robot Cartesiano (PPP)
disp('=== Robot Cartesiano ===');
p_Cartesian = [q(1); q(2); q(3)];
fprintf('Posizione Cartesiano: [%.2f, %.2f, %.2f]\n', p_Cartesian);

%% Visualizzazione 3D
subplot(2,4,1); plotRobotRRR(q, L1, L2, L3); title('RRR');
subplot(2,4,2); plotRobotRPR(q, L1, L3); title('RPR');
subplot(2,4,3); plotRobotPRR(q, L2, L3); title('PRR');
subplot(2,4,4); plotRobotSCARA(q, L1, L2); title('SCARA');
subplot(2,4,5); plotRobotCylindrical(q); title('Cilindrico');
subplot(2,4,6); plotRobotSpherical(q); title('Sferico');
subplot(2,4,7); plotRobotCartesian(q); title('Cartesiano');

%% Funzioni di plotting (da aggiungere alla fine dello script)
function plotRobotRRR(q, L1, L2, L3)
    % Implementa il plotting per RRR
    hold on; axis equal; grid on;
    xlim([-3 3]); ylim([-3 3]); zlim([0 2]);
    view(3);
    % Disegna i bracci
    plot3([0 L1*cos(q(1))], [0 L1*sin(q(1))], [0 0], 'r-o', 'LineWidth', 2);
    plot3([L1*cos(q(1)) L1*cos(q(1))+L2*cos(q(1)+q(2))], ...
          [L1*sin(q(1)) L1*sin(q(1))+L2*sin(q(1)+q(2))], ...
          [0 0], 'g-o', 'LineWidth', 2);
    plot3([L1*cos(q(1))+L2*cos(q(1)+q(2)) L1*cos(q(1))+L2*cos(q(1)+q(2))+L3*cos(q(1)+q(2)+q(3))], ...
          [L1*sin(q(1))+L2*sin(q(1)+q(2)) L1*sin(q(1))+L2*sin(q(1)+q(2))+L3*sin(q(1)+q(2)+q(3))], ...
          [0 0], 'b-o', 'LineWidth', 2);
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

function plotRobotRPR(q, L1, L3)
    % Implementa il plotting per RPR
    hold on; axis equal; grid on;
    xlim([-3 3]); ylim([-3 3]); zlim([0 2]);
    view(3);
    % Disegna i bracci
    plot3([0 (L1+q(2))*cos(q(1))], [0 (L1+q(2))*sin(q(1))], [0 0], 'r-o', 'LineWidth', 2);
    plot3([(L1+q(2))*cos(q(1)) (L1+q(2))*cos(q(1))], ...
          [(L1+q(2))