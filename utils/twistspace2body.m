function V_b = twistspace2body(V_s,T)
    % Given a space frame {s} and a moving body frame {b}, calculate the spacial velocity (twist) Vb in the body frame.
    % Inputs: The spatial velocity V expressed in the space frame, The homogeneous transformation matrix T between {s} and {b}
   
    %Unpack transformation matrix
    R = [T(1:3,1:3)];
    p = [T(1:3,4)];
    
    % Generate adjoint transformation matrix
    pbracket = skew(p);
    topright = zeros(3);
    
    Adt = [R topright
           pbracket * R R];
    
    V_b = pinv(Adt) * V_s;
    
end