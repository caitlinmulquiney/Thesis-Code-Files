function [F] = System(t, Fo)
% clc;

%{b} is defined with origin at free surface, below mastfoot


% Preliminary computations to come close to equilibrium

eta0 = Fo(1:6);
nu0 = Fo(7:12);
wind.speedInN = 9.231; % wind speed in m/s
wind.direction = 30*pi/180; %(propagation direction, positive=wind from port side) 60deg= "-120" in classical terms
wave=[]; % no waves for now, but it will come

foil=loadFoilDescription;
totalLoad = zeros(6,1);
for idxFoil=1:length(foil)
    totalLoad = totalLoad + foilLoad(eta0,nu0,foil{idxFoil},wind,wave,false);
end
totalLoad= totalLoad + weightLoad(eta0,false);
totalLoad = totalLoad + aerodynamicLoadSuperstructure(eta0,nu0,wind,false);

M = massDistribTrimaran(false);
C = coriolisCentripetal(M,nu0);
nud = M\(totalLoad - C*nu0);
nud(1) = 0;
F = [Jbn(eta0)*nu0; nud];
t
return