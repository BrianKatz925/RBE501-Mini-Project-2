% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
% Brian Katz
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/22/2022

%% Given a Twist V = S * theta, where S is a 6 dimensional vector is a
%%% Screw Axis and theta is a scalar, write a function that calculates the
%%% corresponding homogeneous transformation matrix T which is a special
%%% euclidian matrix
function T = twist2ht(S,theta)
    % Transformation matrix = exp([S] * theta)
    % Prepare components of transformation matrix
    % S is a screw axis matrix (S = [w v]^T that consists of two components, omega and v where omega is the rotation and translation -> creating tangential velocity by virtue of rotation
    % and v is simple translation
    omega = [S(1); S(2); S(3);];
    v = [S(4); S(5); S(6);]; 
    % To calculate R, we convert exponential coordinates to a rotation matrix using the previous derived function and rodrigues' formula
    R = axisangle2rot(omega,theta); 
    
    I = [1 0 0; % identity matrix
        0 1 0;
        0 0 1;];
    
    omegaScew = [0 -omega(3) omega(2); % apply bracket operator to omega
                omega(3) 0 -omega(1);
                -omega(2) omega(1) 0;];
  
    p = (I * theta + (1 - cos(theta)) * omegaScew + (theta - sin(theta)) * omegaScew^2) * v;  % generate position matrix using Rodrigues' Formula
    
    T = [R(1,1) R(1,2) R(1,3) p(1);
         R(2,1) R(2,2) R(2,3) p(2);
         R(3,1) R(3,2) R(3,3) p(3);
         0      0      0      1;];
    %NOTE: you do not need to take care of the special case where the magnitudes of v = 1 and w = 0, progrmaming takes it into account mathematically
end