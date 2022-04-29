function S_b = space2body(S_s,T)    
    S_b = [];
    for i = 1:size(S_s, 2)
        S_b = [S_b twistspace2body(S_s(:, i), T)];
    end
end
function V_b = twistspace2body(V_s,T)
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    adT = [R zeros(3, 3);
            skew(p)*R R];
    
    V_b = pinv(adT) * V_s;
end