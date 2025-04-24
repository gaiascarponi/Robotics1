function S = S_omega(omega)
    % Calcola la matrice antisimmetrica S(omega) associata al vettore omega
    % Input: omega = [omega_x; omega_y; omega_z] (vettore colonna 3x1)
    % Output: Matrice S 3x3
    
    omega_x = omega(1);
    omega_y = omega(2);
    omega_z = omega(3);
    
    S = [0          -omega_z    omega_y;
         omega_z    0          -omega_x;
         -omega_y   omega_x    0];
    
    % Verifica che S sia antisimmetrica (S = -S')
    if ~isequal(S, -S')
        warning('La matrice generata non è antisimmetrica. Controlla l''input.');
    end
end