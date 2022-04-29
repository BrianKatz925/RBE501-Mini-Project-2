function M = MassMatrixCalculator(currentQ, S, M, G)
    disp(size(currentQ,1));
    Jib = zeros(6,size(currentQ,1)); % define initial body jacobian and mass matrix sizes
    Mtemp = zeros(size(currentQ,1),size(currentQ,1));
    for i = 1:1:6
        disp(S(:,1:i));
        disp(M(:,:,1:i));
        Jib = jacobe(S(:,1:i), M(:,:,1:i), currentQ); % iteratively calculate body jacobian
        disp(Jib);
        Mtemp = Mtemp + Jib' * G(:,:,i) * Jib;
    end
    M = Mtemp;
    disp(M);
end