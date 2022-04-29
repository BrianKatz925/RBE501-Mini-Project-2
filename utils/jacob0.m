% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
% Brian Katz
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/22/2022

%% Write function jacob0 to calculate the Jacobian matrix for a robot arm. Must be calculated in space frame. 
% Function takes input matrix S containing all screw axes (by column) and vector of joint variables q
% Works for kinematic chains with any number of n joints
function J = jacob0(S,q) 
    transMatArray = cell(1, size(S,2)); % create an empty array of the size of the number of screw axes - columns in S matrix
    adtMatArray = cell(1, size(S,2)); % list of adjoint transformation matrices should be same size as homogeneous trans. matrix size
    J = zeros(size(S, 1), size(q, 2)); % empty jacobian array of size: dimension of screw axis twist by number of joint variables
    currTMultiplication = eye(4); % set up transformation matrix for concatenating transformations
    for i= 1:1:size(S,2) % for each screw axis twist (separated by column)
        currScrewAxis = S(:, i); % extract one screw axis at a time by column
        % extract components from twist
        currOmega = [currScrewAxis(1); currScrewAxis(2); currScrewAxis(3);];
        currV = [currScrewAxis(4); currScrewAxis(5); currScrewAxis(6);];
        % create homogeneous transformation matrix from twist - need to use full screw axis and joint angle as parameters, not screw axis components
        transMatArray{i} = twist2ht(currScrewAxis, q(i)); 
        if i == 1 
            J(:,1) = currScrewAxis; % first column of jacobian will always be the first screw axis since there is no adjoint transformation
            currTMultiplication = transMatArray{i}; % store first transformation matrix
        else
            currTMultiplication = currTMultiplication * transMatArray{i}; % perform transformation matrix multiplication for each previous joint
            
            % create adjoint transformation using adjoint transformation matrix function
            adtMatArray{i} = adjointTwist(currScrewAxis, currTMultiplication);  
            J(:, i) = adtMatArray{i};
        end
    end  
end