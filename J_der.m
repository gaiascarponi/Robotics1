function J_dot = J_der(J, q, qdot)
% J_dot_total - Calcola simbolicamente la derivata totale della Jacobiana
%
% INPUTS:
%   J           : Jacobiana simbolica [m x n]
%   q           : vettore simbolico delle variabili articolari [q1; ...; qn]
%   qdot        : vettore simbolico delle derivate [qdot1; ...; qdotn]
%
% OUTPUT:
%   J_dot_q_dot : derivata totale nel tempo della Jacobiana (m x n)

    [m, n] = size(J);
    J_dot = sym(zeros(m, n));

    for i = 1:n
        dJ_dqi = diff(J, q(i));
        J_dot = J_dot + dJ_dqi * qdot(i);
    end

    J_dot = simplify(J_dot);
end

