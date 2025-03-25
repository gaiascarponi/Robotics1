function[T]=T_phi_EULER(axes,angles)
% given a sequence of rotations (EULER!!!) and their corresponding 
% angles, this function computes the matrix T(Φ) needed to the computation 
% of ω, through :    ω = T(Φ)*Φ_dot

% initialize variables
T=[];
R1=[];
R2=[];
syms x y z

% construction of T
for i=1:3
    switch axes(i)
        case {"x", "X"}
            
            if i==1 
                T = [1 0 0]';
                R1=element_Rot(sym(x),angles(i));
            end
            if i==2
                T2 = R1(:,1);
                R2=R1*element_Rot(sym(x),angles(i));
            end
            if i==3
                T3 = R2(:,1);
            end

       case {"y", "Y"}

            if i==1 
                T = [0 1 0]';
                R1=element_Rot(sym(y),angles(i));
            end
            if i==2
                T2 = R1(:,2);
                R2=R1*element_Rot(sym(y),angles(i));
            end
            if i==3
                T3 = R2(:,2);
            end

        case {"z", "Z"}
            if i==1
                T= [0 0 1]';
                R1=element_Rot(sym(z),angles(i));
            end
            if i==2
                T2 = R1(:,3);
                R2=R1*element_Rot(sym(z),angles(i));
            end
            if i==3
                T3 = R2(:,3);
            end

    end
end
T=[T T2 T3];