%1- NULL SPACE;
null(J)
%2- RANGE SPACE;
% Se J è numerica
R = orth(J); % Calcola una base ortonormale per il range
% Se J è simbolica
R = colspace(J); % Calcola una base per il range
% Visualizzazione del range
disp('Range (R(J)):');
disp(R);


[R, pivots] = rref(J);

% Base del range space (colonne pivot)
%range_space = J(:, pivots);


%
range_space = orth(J);
%compl.
null(transpose(J1))