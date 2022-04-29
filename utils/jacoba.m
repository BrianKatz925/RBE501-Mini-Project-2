function J_a = jacoba(S,M,q)    
% Calculates the analytical Jacobian matrix Ja, which provides the mapping between joint variables q and the end effector Cartesian velocity p dot expressed in the space frame.
 J_b = jacobe(S,M,q); % calculate the body jacobian
 J_vb = J_b(4:6, :); % remove the top three rows of the body Jacobian
 
  Thomebody = fkine(S,M,q,'space'); % calculate the transformation matrix from the space frame to the body frame
    
  % unpack the transformation matrix and get the rotation between the space and body frame
  R = [Thomebody(1:3,1:3)];
  
  % multiply by the rotation matrix to get cartesian velocities at end
  % effector
  J_a = R * J_vb;
 
end