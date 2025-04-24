function [r1, r2, theta1, theta2, sin_theta_pos, sin_theta_neg, cos_theta] = axis_angle_inv(ARB)
    % Calcolo di cos(theta) dalla traccia
    cos_theta = (trace(ARB) - 1) / 2;
    cos_theta = max(min(cos_theta, 1), -1); % Evita errori numerici

    % Calcolo di sin(theta) (valore assoluto)
    sin_theta_abs = sqrt((ARB(3,2) - ARB(2,3))^2 + (ARB(1,3) - ARB(3,1))^2 + (ARB(2,1) - ARB(1,2))^2) / 2;

    % Due possibili valori per sin(theta): positivo e negativo
    sin_theta_pos = sin_theta_abs;
    sin_theta_neg = -sin_theta_abs;

    % Due possibili angoli corrispondenti
    theta1 = atan2(sin_theta_pos, cos_theta);
    theta2 = atan2(sin_theta_neg, cos_theta);

    % Caso speciale: theta = 0 (matrice identità, asse indefinito)
    if abs(theta1) < 1e-6 || abs(theta2) < 1e-6
        warning('Theta is 0: rotation axis is undefined (any axis is valid).');
        r1 = [1; 0; 0]; % Asse arbitrario
        r2 = [1; 0; 0]; % Asse arbitrario
        return;
    end

    % Caso speciale: theta = ±pi (sin(theta) = 0, ma soluzione unica per r)
    if abs(abs(theta1) - pi) < 1e-6 || abs(abs(theta2) - pi) < 1e-6
        warning('Theta is ±pi: rotation axis is unique but requires special computation.');
        
        % Calcolo dell'asse di rotazione per theta = ±pi
        r1 = [sqrt((ARB(1,1) + 1)/2); 
              sqrt((ARB(2,2) + 1)/2); 
              sqrt((ARB(3,3) + 1)/2)] .* sign([ARB(1,2) - ARB(2,1); 
                                              ARB(1,3) - ARB(3,1); 
                                              ARB(2,3) - ARB(3,2)]);
        r1 = r1 / norm(r1);
        r2 = r1; % Soluzione unica in questo caso
        return;
    end

    % Caso generale (theta ≠ 0, theta ≠ ±pi)
    if abs(sin_theta_pos) > 1e-6
        r1 = [ARB(3,2) - ARB(2,3); 
              ARB(1,3) - ARB(3,1); 
              ARB(2,1) - ARB(1,2)] / (2 * sin_theta_pos);
        r1 = r1 / norm(r1);
    else
        % Questo blocco non dovrebbe essere raggiunto grazie ai casi speciali sopra
        r1 = [1; 0; 0]; % Asse arbitrario (backup)
    end

    if abs(sin_theta_neg) > 1e-6
        r2 = [ARB(3,2) - ARB(2,3); 
              ARB(1,3) - ARB(3,1); 
              ARB(2,1) - ARB(1,2)] / (2 * sin_theta_neg);
        r2 = r2 / norm(r2);
    else
        r2 = [1; 0; 0]; % Asse arbitrario (backup)
    end
end