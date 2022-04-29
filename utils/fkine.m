% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
% Brian Katz
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/22/2022

%% Write a function fkine to calculate the forward kinematics of a robotic
%%% arm using the Product of Exponentials Formula. Takes input matrix S
%%% containing all the screw axes (each column is a screw axis, with column
%%% 1 being the screw axis for the first joint, column 2 being the screw
%%% axis for the second joint, and so on), a vector of joint variables q,
%%% and a homogeneous transformation matrix M representing the robot pose
%%% in its home configuration
function T = fkine(S,M,q,frame)
    % Modify fkine to accept an additional parameter that controls whether the product of exponentials is calculated in the space or body frame of the robot.
    % S, M, q are the same as homework 2
    % frame is a string 'body' or 'space'
        Tarray = cell(1, size(S, 2)); 
    
        % create an identity matrix of transformations ready to multiply - do not make this zeros!
        T = eye(4); 
    
        % loop through all screw axes
        for i=1:1:size(S,2)
    
            % First, extract all transformation matrices taking the form of exp([S(i)] * theta(i))
            Tarray{i} = twist2ht(S(:, i), q(i));
            
            % multiply all joint transformation matrices together
            T = T * Tarray{i};
        end
        
    if strcmp('space',frame) 
        T = T * M; % Multiply by home configuration matrix, M which is given
    elseif strcmp('body',frame)
        T = M * T; % Premultiply by the home configuration matrix in the body frame   
    end
end