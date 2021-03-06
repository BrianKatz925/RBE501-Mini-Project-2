function M = MassMatrixCalculator(currentQ, S, M, G)
    Mtemp = zeros(size(currentQ,1),size(currentQ,1));
    for i = 1:1:size(currentQ,1)
        Jib = zeros(6,size(currentQ,1)); % define initial body jacobian and mass matrix sizes
        disp(S(:,1:i));
        Jib(:,1:i) = jacobe(S(:,1:i), M(:,:,i), currentQ); % iteratively calculate body jacobian padding w/ zeros
        disp(Jib);
        Mtemp = Mtemp + Jib' * G(:,:,i) * Jib; % concatenate Mtemp mass matrix
        disp(Mtemp);
    end
    M = Mtemp;
    disp(M);
end












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

function S = skew(v)
%SKEW Returns the skew-symmetric matrix associated with the 3D vector
%passed as input

if length(v) == 3
    S = [  0   -v(3)  v(2)
        v(3)  0    -v(1)
        -v(2) v(1)   0];
else
    error('argument must be a 3-vector');
end

end

