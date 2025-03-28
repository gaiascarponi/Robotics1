function determinants = DetSubMat2(J)
% INPUT: J - Matrice simbolica mxn (m > n, m <= 6)
% OUTPUT: determinants - Cell array dei determinanti delle sottomatrici kxk (k = n)

[m, n] = size(J);

% Verifica condizioni d'ingresso
if m <= n
    error('La matrice deve avere m > n.');
elseif m > 6
    error('m deve essere <= 6 per evitare calcoli complessi.');
end

k = n; % Dimensione delle sottomatrici quadrate (k = n)
num_submatrices = nchoosek(m, k); % Numero di sottomatrici
determinants = sym(zeros(num_submatrices, 1)); % Preallocazione

% Genera tutte le combinazioni di righe
row_combinations = nchoosek(1:m, k);

for i = 1:num_submatrices
    rows = row_combinations(i, :);
    submatrix = J(rows, :); % Estrai sottomatrice kxn (ma k = n)
    determinants(i) = det(submatrix); % Calcolo simbolico
end

% Mostra risultati
disp(['Determinanti delle sottomatrici ', num2str(k), 'x', num2str(k), ':']);
disp(determinants);
end