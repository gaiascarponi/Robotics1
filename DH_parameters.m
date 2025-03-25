function[T,varargout]=DH_parameters(table) 
% given a sequence of transformation, this function computes the final
% homogeneous matrix T, from frame 0 to n
% INPUT:
% table = just a matrix with the DH parameters (just make sure that all
%       variables are symbolic when given as input in table)
% OUTPUT, 1 requested, 2 optional:
% T = homogeneous matrix (needed)
% A = cell array containing n matrices, each one is the homogeneous matrix
%       up to that index (so you have A_01, A_02, ..., A_0n [so the last 
%       one is literally equal to T !!] ). You might need it for p_0E, 
%       p_1E,... and so on
%CON A HAI 0-1 / 0-2 / 0-3 ....
% A_sing_trans = here I stored each transformation of the sequence in 
%       another cell array(so you have A_01, A_12,...,A_n-1n)
%       CONSECUTIVE!!
% celldisp(A) or whatever I want to print. 


rows = size(table,1);
A = cell(1,rows);
T=1;

for i=1:rows
    theta = table(i,4);
    d = table(i,3);
    a = table(i,2);
    alpha = table(i,1);
    A{i} = [cos(theta) -cos(alpha)*sin(theta) sin(alpha)*sin(theta) a*cos(theta);
            sin(theta) cos(alpha)*cos(theta) -sin(alpha)*cos(theta) a*sin(theta);
            0 sin(alpha) cos(alpha) d;
            0 0 0 1];
    T = T*A{i};
    if nargout==3
        A_single_trans{i} = A{i};
    end
    if nargout~=1
        if i~=1
            A{i} = A{i-1}*A{i};
            A{i} = simplify(A{i});
        end
    end
    T = simplify(T);
end

switch nargout
    case 2
        varargout{1} = A;
    case 3
        varargout{1} = A;
        varargout{2} = A_single_trans;
end

% disp('the final homogeneous matrix is')
% disp(T)
% disp('here are some useful mid transformations ')
% disp(A)