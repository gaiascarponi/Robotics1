function[J]=Geometric_Jacobian(table,joints)

% INPUT:
% joints= ['r' 'p' 'r']
% table= DH parameter table


% initializing variables
n=length(joints);
q=[];
z0=[0 0 1]';
z=z0;

% computing the results needed from direct kinematics (R,T,p)
[T,A]=DH_parameters(table);
for i=1:n
    switch joints(i)
        case {"p", "P"}
            % q(i)= table(i,3);
            if i==1
                JL=[0 0 1]';
                JA=[0 0 0]';
            else
                z= A{i-1}(1:3,1:3)*z0;
                JL=[JL z];
                JA=[JA zeros(3,1)];
            end
        case {"r","R"}
            % q(i)= table(i,4);
            if i==1
                JL = cross(z0,T(1:3,4));
                JA = z0;
            else
                z= A{i-1}(1:3,1:3)*z0;
                p_iE= T(1:3,4)-A{i-1}(1:3,4);
                JL = [JL cross(z,p_iE)];
                JA = [JA z];
            end
    end
end

J=[JL; JA];
J=simplify(J);