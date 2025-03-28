function determinants = DetSubMat(J)
% INPUT: J - Matrice simbolica 3x4 (es. Jacobiana con variabili simboliche)
% OUTPUT: determinants - Determinanti simbolici delle sottomatrici 3x3

[m, n] = size(J);
if m ~= 3 || n ~= 4
    error('La matrice deve essere 3x4');
end

% Inizializza vettore simbolico
determinants = sym(zeros(4, 1));

% Combinazioni di colonne (C(4,3) = 4)
column_combinations = nchoosek(1:4, 3);

for i = 1:4
    cols = column_combinations(i, :);
    submatrix = J(:, cols);
    determinants(i) = det(submatrix); % Calcolo simbolico
end

% Mostra risultati
disp('Determinanti simbolici delle sottomatrici 3x3:');
disp(determinants);
end