function [q, v, a, j, t, T] = CubicTrajectory(q0, delta_q_or_qf, varargin)
    % qi(t)= ....

    % Input:   
    % - q0: posizione iniziale (SE NON LA HAI METTI SIMBOLICA (idem per delta)!!!)
    % - delta_q_or_qf: delta_q (se 'normalized') o qf (altrimenti)

    % Parametri aggiuntivi:
    %   * 'normalized': attiva la modalità doubly normalized
    %   * 'T': tempo finale

    % Output:
    % -q= q(t)
    % -a= qdot(t)
    % -v= qdotdot(t)

    % Come fare se VINCOLI (V | A)   ///  NB= T/2=0.5  ///
    %   * 'max velocity', V: vincolo di velocità --> subs(v, ['tau'], [0.5])
    %                                            supponi quindi che il
    %                                            massimo della velocità lo
    %                                            raggiunga al 50% del
    %                                            tempo
    %   * 'acceleration', A: vincolo di accelerazione --> subs(a, ['tau'], [T/2])
    %                                            supponi che quando
    %                                            accelerazione 0 allora
    %                                            massima velocità

    % --- Parsing degli input ---
    normalized = any(strcmpi(varargin, 'normalized'));
    
    if normalized
        delta_q = delta_q_or_qf;
    else
        delta_q = delta_q_or_qf - q0;
    end

    % --- Gestione tempo T e vincoli ---
    T = [];
    V = [];
    A = [];
    
    for i = 1:length(varargin)
        if strcmpi(varargin{i}, 'velocity') && i < length(varargin)
            V = varargin{i+1};
        elseif strcmpi(varargin{i}, 'acceleration') && i < length(varargin)
            A = varargin{i+1};
        elseif (isnumeric(varargin{i}) || isa(varargin{i}, 'sym')) && isempty(T) && ...
               ~any(strcmpi(varargin{i}, {'normalized', 'velocity', 'acceleration'}))
            T = varargin{i};
        end
    end

    % --- Calcolo automatico di T se non specificato ---
    if isempty(T)
        if ~isempty(V)
            T = (3 * abs(delta_q)) / (2 * V);
        elseif ~isempty(A)
            T = sqrt((6 * abs(delta_q)) / A);
        else
            if isa(q0, 'sym') || isa(delta_q, 'sym')
                T = sym('T');
            else
                error('Specificare T o un vincolo (V/A).');
            end
        end
    end

    % --- Generazione traiettoria ---
    if normalized
        % Doubly normalized (tau ∈ [0,1])
        tau = sym('tau');
        t = tau * T;
        q = q0 + delta_q * (3 * tau^2 - 2 * tau^3);
        v = (delta_q / T) * (6 * tau * (1 - tau));
        a = (delta_q / T^2) * (6 * (1 - 2 * tau));
        j = -12 * delta_q / T^3;
    else
        % Non-normalizzata (t ∈ [0,T])
        t = sym('t');
        A_mat = [1, 0,     0,       0; 
                 0, 1,     0,       0; 
                 1, T,     T^2,     T^3; 
                 0, 1,     2*T,     3*T^2];
        b_vec = [q0; 0; q0 + delta_q; 0];
        coeff = A_mat \ b_vec;
        q = coeff(1) + coeff(2)*t + coeff(3)*t^2 + coeff(4)*t^3;
        v = coeff(2) + 2*coeff(3)*t + 3*coeff(4)*t^2;
        a = 2*coeff(3) + 6*coeff(4)*t;
        j = 6*coeff(4);
    end
end