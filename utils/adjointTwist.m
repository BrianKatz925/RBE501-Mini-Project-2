% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
% Brian Katz
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/22/2022

%%% Given a Twist V expressed with respect to some arbitrary reference
%%% frame, calculate its representation in a new frame whose position and
%%% orientation is described by a homogeneous transformation matrix T.
function twist_inB = adjointTwist(twist_inA,T_AB)
    % pulling rotation and position matrix from given homogeneous T matrix
    R = [T_AB(1, 1:3);
        T_AB(2, 1:3);
        T_AB(3, 1:3);];
    p = [T_AB(1:3, 4)];
    pScew = [0 -p(3) p(2);  % convert to scew symmetrix matrix
             p(3) 0 -p(1);
            -p(2) p(1) 0;];
    bL = pScew * R; % bottom left term of adjoint transform matrix - [p] * R
    % Create adjoint transform matrix
    Adt = [R(1,:) 0    0    0;
          R(2,:) 0    0    0;
          R(3,:) 0    0    0;
          bL(1,:)   R(1,:);
          bL(2,:)   R(2,:);
          bL(3,:)   R(3,:);];
    % Perform transformation
    % Using this method, we don't have to use the bracket operator on the twist and then undo it to get our transformed twist, 
    % we can just directly convert twists knowing the transformation matrix
    twist_inB = Adt * twist_inA;       
end