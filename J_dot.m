function [J_dot, J_dot_q_dot] = J_der(J, q, qdot)
% J_dot - Calcola simbolicamente:
%   (1) la derivata parziale della Jacobiana rispetto a q -> J_dot
%   (2) la derivata totale nel tempo J_dot * qdot         -> J_dot_q_dot
%
% INPUTS:
%   J            : Jacobiana simbolica [m x n]
%   q            : vettore simbolico delle variabili articolari [q1; ...; qn]
%   qdot         : vettore simbolico delle derivate [qdot1; ...; qdotn]
%
% OUTPUTS:
%   J_dot        : array simbolico [m x n x n], con J_dot(:,:,i) = ∂J/∂q_i
%   J_dot_q_dot  : matrice [m x n] con la derivata totale: sum_i ∂J/∂q_i * qdot_i

    [m, n] = size(J);

    % Preallocazioni
    J_dot = sym(zeros(m, n, n));
    J_dot_q_dot = sym(zeros(m, n));

    % Ciclo per costruire ∂J/∂q_i e calcolare J_dot * qdot
    for i = 1:n
        dJ_dqi = diff(J, q(i));
        J_dot(:, :, i) = simplify(dJ_dqi);               % salva derivata parziale
        J_dot_q_dot = J_dot_q_dot + dJ_dqi * qdot(i);    % accumula J_dot * qdot
    end

    % Semplifica il risultato finale
    J_dot_q_dot = simplify(J_dot_q_dot);
end
