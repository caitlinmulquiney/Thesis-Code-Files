function residual = computeResidual(x,eta,nu,foil,wind)
% eta = [0;0;x(3);x(4);x(5);0];
% nu = [[x(1)*cos(x(2));x(1)*sin(x(2));0]; zeros(3,1)];
% nu = [Rbn(eta).'* [x(1)*cos(x(2));x(1)*sin(x(2));0]; zeros(3,1)];

% Adjust L foil
foil{2}.attitudeInB = foil{2}.attitudeInB + [0;x(1);0];
foil{3}.attitudeInB = foil{3}.attitudeInB + [0;x(1);0];
% Adjust rudder
foil{4}.attitudeInB = foil{4}.attitudeInB + [0;x(2);x(3)]; 
foil{8}.attitudeInB = foil{8}.attitudeInB + [0;x(2);x(3)];
% Adjust T-foils rudders
foil{5}.attitudeInB = foil{5}.attitudeInB + [0;0;x(3)];
foil{9}.attitudeInB = foil{9}.attitudeInB + [0;0;x(3)];
% Adjust T-foils centerboard
foil{6}.attitudeInB = foil{6}.attitudeInB + [0;x(4);0];

totalLoad = zeros(6,1);
for idxFoil=1:length(foil)
totalLoad = totalLoad + foilLoad(eta,nu,foil{idxFoil},wind,[],false);
end
totalLoad= totalLoad + weightLoad(eta,false);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta,nu,wind,false);

w1 = 1;
w2 = 1;
residual = w1*sqrt(sum((1e-3*totalLoad(1:3)).^2)) + w2*sqrt(sum((1e-3*totalLoad(4:6)).^2));
end