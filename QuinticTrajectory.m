function [q, v, a, j, s, t, T] = QuinticTrajectory(q0, delta_q_or_qf, varargin)
    % Input:
    % - q0: initial position (can be symbolic)
    % - delta_q_or_qf: delta_q (if 'normalized') or qf (otherwise)

    % Additional parameters:
    %   * 'normalized': activates doubly normalized mode
    %   * 'T': final time
    %   * 'velocity', V: velocity constraint
    %   * 'acceleration', A: acceleration constraint
    %   * 'jerk', J: jerk constraint

    % Output:
    % - q: position trajectory q(t)
    % - v: velocity trajectory qdot(t)t
    % - a: acceleration trajectory qdotdot(t)
    % - j: jerk trajectory
    % - s: snap trajectory
    % - t: time variable
    % - T: final time

    % --- Input parsing ---
    normalized = any(strcmpi(varargin, 'normalized'));
    
    if normalized
        delta_q = delta_q_or_qf;
    else
        delta_q = delta_q_or_qf - q0;
    end

    % --- Handle time T and constraints ---
    T = [];
    V = [];
    A = [];
    J = [];
    
    for i = 1:length(varargin)
        if strcmpi(varargin{i}, 'velocity') && i < length(varargin)
            V = varargin{i+1};
        elseif strcmpi(varargin{i}, 'acceleration') && i < length(varargin)
            A = varargin{i+1};
        elseif strcmpi(varargin{i}, 'jerk') && i < length(varargin)
            J = varargin{i+1};
        elseif (isnumeric(varargin{i}) || isa(varargin{i}, 'sym')) && isempty(T) && ...
               ~any(strcmpi(varargin{i}, {'normalized', 'velocity', 'acceleration', 'jerk'}))
            T = varargin{i};
        end
    end

    % --- Automatic T calculation if not specified ---
    if isempty(T)
        if ~isempty(V)
            T = (15 * abs(delta_q)) / (8 * V);
        elseif ~isempty(A)
            T = sqrt((10 * abs(delta_q)) / (sqrt(3) * A));
        elseif ~isempty(J)
            T = nthroot((60 * abs(delta_q)) / (8 * J), 3);
        else
            if isa(q0, 'sym') || isa(delta_q, 'sym')
                T = sym('T');
            else
                error('Specify T or a constraint (V/A/J).');
            end
        end
    end

    % --- Trajectory generation ---
    if normalized
        % Doubly normalized (tau ∈ [0,1])
        tau = sym('tau');
        t = tau * T;
        q = q0 + delta_q * (10 * tau^3 - 15 * tau^4 + 6 * tau^5);
        v = (delta_q / T) * (30 * tau^2 - 60 * tau^3 + 30 * tau^4);
        a = (delta_q / T^2) * (60 * tau - 180 * tau^2 + 120 * tau^3);
        j = (delta_q / T^3) * (60 - 360 * tau + 360 * tau^2);
        s = (delta_q / T^4) * (-360 + 720 * tau);
    else
        % Non-normalized (t ∈ [0,T])
        t = sym('t');
        A_mat = [1, 0,     0,       0,       0,        0; 
                 0, 1,     0,       0,       0,        0; 
                 0, 0,     2,       0,       0,        0; 
                 1, T,     T^2,     T^3,     T^4,      T^5; 
                 0, 1,     2*T,     3*T^2,   4*T^3,    5*T^4; 
                 0, 0,     2,       6*T,     12*T^2,   20*T^3];
        b_vec = [q0; 0; 0; q0 + delta_q; 0; 0];
        coeff = A_mat \ b_vec;
        q = coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3 + coeff(5)*t^4 + coeff(6)*t^5;
        v = coeff(2) + 2*coeff(3)*t + 3*coeff(4)*t^2 + 4*coeff(5)*t^3 + 5*coeff(6)*t^4;
        a = 2*coeff(3) + 6*coeff(4)*t + 12*coeff(5)*t^2 + 20*coeff(6)*t^3;
        j = 6*coeff(4) + 24*coeff(5)*t + 60*coeff(6)*t^2;
        s = 24*coeff(5) + 120*coeff(6)*t;
    end
end