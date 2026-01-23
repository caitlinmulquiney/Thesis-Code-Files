function [F] = System(t, Fo)
% clc;

%{b} is defined with origin at free surface, below mastfoot


% Preliminary computations to come close to equilibrium

eta0 = Fo(1:6);
nu0 = Fo(7:12);
wind.speedInN = 10; % wind speed in m/s
wind.direction = 60*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come
verbose = false;

foil=loadFoilDescription;
totalLoad = zeros(6,1);
for idxFoil=1:length(foil)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{idxFoil},wind,wave,true);
end
totalLoad= totalLoad + weightLoad(eta0,true);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,true);
M = massDistribTrimaran(verbose);
C = coriolisCentripetal(M,nu0);

nud = M^(-1)*(totalLoad - C*nu0);
F = [nu0; nud];
return