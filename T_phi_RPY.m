function[T]=T_phi_RPY(axes,angles)
% given a sequence of rotations (RPY!!!) and their corresponding 
% angles, this function computes the matrix T(Φ) needed to the computation 
% of ω, through :    ω = T(Φ)*Φ_dot (phi_dot=(alpha_dot, beta_dot, gamma_dot))
%subs(T_PHI,[beta gamma],[0 pi/2])

% initialize variables
S=[axes(3) axes(2) axes(1)];
s=[angles(3) angles(2) angles(1)];
R1=[];
R2=[];
syms x y z

% construction of T
for i=1:3
    switch S(i)
        case {"x", "X"}
            switch i
                case 1
                    T1 = [1 0 0]';
                    R1=element_Rot(sym(x),s(i));
                case 2
                    T2 = R1(:,1);
                    R2=R1*element_Rot(sym(x),s(i));
                case 3
                    T3 = R2(:,1);
            end

       case {"y", "Y"}
           switch i
               case 1 
                    T1 = [0 1 0]';
                    R1=element_Rot(sym(y),s(i));
               case 2
                    T2 = R1(:,2);
                    R2=R1*element_Rot(sym(y),s(i));
               case 3
                    T3 = R2(:,2);
            end

        case {"z", "Z"}

            switch i
                case 1
                    T1 = [0 0 1]';
                    R1=element_Rot(sym(z),s(i));
                case 2
                    T2 = R1(:,3);
                    R2=R1*element_Rot(sym(z),s(i));
                case 3
                    T3 = R2(:,3);
            end

    end
end

% creating the output matrix T
T=[T3 T2 T1];
