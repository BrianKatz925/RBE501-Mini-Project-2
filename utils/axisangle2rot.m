% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
% Brian Katz
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/22/2022

%% Given the exponential coordinate of rotation w * theta, where theta is
%%% a scalar and omega is a 3 dimensional vector with its magnitude being
%%% 1, calculate the corresponding rotation matrix R which is a special
%%% orthogonal 3x3 matrix
function R = axisangle2rot(omega,theta)
    % Convert omega to a scew-symmetric matrix of size 3x3
    omegaScewSym = [0 -omega(3) omega(2);
                    omega(3) 0 -omega(1);
                    -omega(2) omega(1) 0;];
    
    % Apply rodrigues' formula using omega, the vector around which the rotation is happening (axis of rotation + angular rate of rotation) and theta (our joint variable)
    I = [1 0 0;
        0 1 0;
        0 0 1;];
    R = I + sin(theta) * omegaScewSym + (1 - cos(theta)) * omegaScewSym^2;
end