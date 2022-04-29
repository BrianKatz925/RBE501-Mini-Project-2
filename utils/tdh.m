%%% Returns corresponding 4x4 homogeneous transformation matrix given DH
%%% parameters theta, d, a, alpha
function T = tdh(theta, d, a, alpha)
    %% your code here
    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0          sin(alpha)            cos(alpha)                 d;
        0              0                      0                     1;];
end