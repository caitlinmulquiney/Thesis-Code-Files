function drawBoat(eta,nu,foil,wind,wave)
% draws the forces, etc... that applied on the boat
%NB: some of the parameters below are repeated (COGpositionInB, 
% applicationPointAeroLoadSuperstructure) in other functions. 
% If changed Remember to change them here too so that sketches are correct.

centralHullLength = 30; 
floaterLength = 30.6;
floaterToCentralHull = 10.5;
COGpositionInB = [-1.3; 0; -2.4];
applicationPointAeroLoadSuperstructure = [0;0;-10];

scalingForces = .4;

% Rotation matrix 
R = Rbn(eta);

% Draw central hull
p1 = eta(1:3) + R*[centralHullLength/2;0;0];
p2 = eta(1:3) + R*[-centralHullLength/2;0;0];
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r','LineWidth',3);

hold on; 

% Draw starboard floater
p1 = eta(1:3) + R*[floaterLength/2;floaterToCentralHull;-.92];
p2 = eta(1:3) + R*[-floaterLength/2;floaterToCentralHull;-.92];
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r','LineWidth',3);
% Draw port floater
p1 = eta(1:3) + R*[floaterLength/2;-floaterToCentralHull;-.92];
p2 = eta(1:3) + R*[-floaterLength/2;-floaterToCentralHull;-.92];
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r','LineWidth',3);

% Draw foils and their forces
for idxFoil = 1:length(foil)
    p1 = eta(1:3) + R*(foil{idxFoil}.positionInB + Rbn([zeros(3,1);foil{idxFoil}.attitudeInB])*[0;-foil{idxFoil}.span/2;0]);
    p2 = eta(1:3) + R*(foil{idxFoil}.positionInB + Rbn([zeros(3,1);foil{idxFoil}.attitudeInB])*[0;foil{idxFoil}.span/2;0]);
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'b','LineWidth',3);
    
    foilIndividualLoad = foilLoad(eta,nu,foil{idxFoil},wind,wave,false);
    p1 = eta(1:3) + R*(foil{idxFoil}.positionInB);
    p2 = eta(1:3) + R*(foil{idxFoil}.positionInB) + scalingForces * 1e-3 * (R * foilIndividualLoad(1:3));
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'b');
    
    text(mean([p1(1) p2(1)]),mean([p1(2) p2(2)]),mean([p1(3) p2(3)]),num2str(idxFoil),'FontSize',12,'Color','b')
end

% Draw weight
weightIndividualLoad = weightLoad(eta,false);
p1 = eta(1:3) + R*COGpositionInB ;
p2 = eta(1:3) + R*COGpositionInB + scalingForces * 1e-3 * (R * weightIndividualLoad(1:3));
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r  ');

%Draw aerodynamic load on superstructure
aeroLoadInB = aerodynamicLoadSuperstructure(eta,nu,wind,false);
p1 = eta(1:3) + R*applicationPointAeroLoadSuperstructure;
p2 = eta(1:3) + R*applicationPointAeroLoadSuperstructure + scalingForces * 1e-3 * (R * aeroLoadInB(1:3));
plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r  ');

axis equal
set(gca,'zdir','reverse') % more intuitive
set(gca,'ydir','reverse')

end
