clear all;
close all;
clc;

%% Definizione simbolica delle variabili
syms q1 q2 q3 l1 l2 real

% Parametri fissi
l1_val = 1;     % [m]
l2_val = 0.5;   % [m]

% Pose desiderata (simbolica per analisi generica)
px = sym('px', 'real');
py = sym('py', 'real');
alpha = sym('alpha', 'real');

%% Cinematica diretta simbolica (con twist di pi/2)
theta = q1 + q2;
p_x = l1*cos(q1) + l2*cos(theta) + q3*cos(theta + pi/2);
p_y = l1*sin(q1) + l2*sin(theta) + q3*sin(theta + pi/2);
alpha_eq = theta + pi/2;  % Orientamento end-effector

% Semplificazioni trigonometriche
p_x = simplify(p_x);  % Diventa: l1*cos(q1) + l2*cos(q1+q2) - q3*sin(q1+q2)
p_y = simplify(p_y);  % Diventa: l1*sin(q1) + l2*sin(q1+q2) + q3*cos(q1+q2)

%% Risoluzione simbolica dell'inversa
% Passo 1: Ricava theta = q1 + q2 dall'orientamento
theta_sol = alpha - pi/2;

% Passo 2: Sostituisci theta nelle equazioni di posizione
eq1 = l1*cos(q1) + l2*cos(theta_sol) - q3*sin(theta_sol) == px;
eq2 = l1*sin(q1) + l2*sin(theta_sol) + q3*cos(theta_sol) == py;

% Risolvi per q1 e q3
sol = solve([eq1, eq2], [q1, q3], 'ReturnConditions', true);

% Mostra le soluzioni simboliche
disp('Soluzioni per q1:');
disp(sol.q1);
disp('Soluzioni per q3:');
disp(sol.q3);

% Condizioni per esistenza della soluzione
disp('Condizioni per la soluzione:');
disp(sol.conditions);

%% Sostituzione numerica per r = [1, 1, pi/4]
fprintf('\n--- Soluzione numerica per r = [1, 1, pi/4] ---\n');
r_num = [1; 1; pi/4];
alpha_num = r_num(3);

% Calcola theta numerico
theta_num = alpha_num - pi/2;

% Sostituisci nei parametri simbolici
eq1_num = subs(eq1, {px, py, alpha, l1, l2}, {r_num(1), r_num(2), alpha_num, l1_val, l2_val});
eq2_num = subs(eq2, {px, py, alpha, l1, l2}, {r_num(1), r_num(2), alpha_num, l1_val, l2_val});

% Risolvi numericamente
[q1_num, q3_num] = vpasolve([eq1_num, eq2_num], [q1, q3]);

% Calcola q2
q2_num = theta_num - q1_num;

% Mostra i risultati
fprintf('q1 = %.4f rad (%.2f°)\n', double(q1_num), rad2deg(double(q1_num)));
fprintf('q2 = %.4f rad (%.2f°)\n', double(q2_num), rad2deg(double(q2_num)));
fprintf('q3 = %.4f m\n', double(q3_num));

%% Jacobiana simbolica
J = jacobian([p_x; p_y; alpha_eq], [q1, q2, q3]);
disp('Jacobiana simbolica:');
disp(J);

% Valutazione numerica della Jacobiana
J_num = subs(J, {q1, q2, q3, l1, l2}, {q1_num, q2_num, q3_num, l1_val, l2_val});
fprintf('\nJacobiana numerica:\n');
disp(double(J_num));

% Verifica singolarità
if rank(J_num) < 3
    fprintf('Configurazione singolare!\n');
else
    fprintf('Configurazione regolare (det(J) = %.4f)\n', det(J_num));
end