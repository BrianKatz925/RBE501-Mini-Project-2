function J_b = jacobe(S,M,q)    
%jacobe calculates the body Jacobian matrix expressed in the end effector frame for a robotic arm
% It takes an input matrix S containing all the screw axes and a vector of joint variables q. The screw axes are expressed in the space frame.
    J_s = jacob0(S, q); % calculate the space jacobian in the space frame
    Thomebody = fkine(S,M,q,'space'); % calculate the transformation matrix from the space frame to the body frame
    
    % unpack the transformation matrix and calculate the adjoint transformation matrix
    R = [Thomebody(1:3,1:3)];
    p = [Thomebody(1:3,4)];
    pbracket = skew(p);
    topright = zeros(3);
    
    Adt = [R topright
           pbracket * R R];
    
    % calculate the body jacobian using the inverse of the adjoint transformation
    J_b = pinv(Adt) * J_s;
    
end
