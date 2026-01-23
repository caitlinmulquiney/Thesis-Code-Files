function drawBoat(eta,nu,foil,wind,wave)
% draws the forces, etc... that applied on the boat
% NB: some of the parameters below are repeated (COGpositionInB,
% applicationPointAeroLoadSuperstructure) in other functions.
% If changed Remember to change them here too so that sketches are correct.

hullLength = 15;
hullSeparation = 3.744*2; % distance between port and starboard hulls
COGpositionInB = [5.92896; 0.07904; -3.86752];
applicationPointAeroLoadSuperstructure = [5.92896;0;-12];
scalingForces = .4;

% Rotation matrix
R = Rbn(eta);

% Draw starboard hull
p1 = eta(1:3) + R*[hullLength; hullSeparation/2; 0];
p2 = eta(1:3) + R*[0; hullSeparation/2; 0];
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r', 'LineWidth', 3);
hold on;

% Draw port hull
p1 = eta(1:3) + R*[hullLength; -hullSeparation/2; 0];
p2 = eta(1:3) + R*[0; -hullSeparation/2; 0];
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r', 'LineWidth', 3);

% Draw foils and their forces
for idxFoil = 1:length(foil)
    % Foil position and orientation
    foilPos = foil{idxFoil}.positionInB;
    foilAtt = foil{idxFoil}.attitudeInB;
    
    % Foil endpoints (along span)
    p1 = eta(1:3) + R*(foilPos + Rbn([zeros(3,1); foilAtt])*[0; -foil{idxFoil}.span/2; 0]);
    p2 = eta(1:3) + R*(foilPos + Rbn([zeros(3,1); foilAtt])*[0; foil{idxFoil}.span/2; 0]);
    
    % Choose color based on foil type
    if contains(foil{idxFoil}.type, 'sail')
        foilColor = 'g';
        lineWidth = 2;
    elseif contains(foil{idxFoil}.type, 'L foil')
        foilColor = 'c';
        lineWidth = 3;
    elseif contains(foil{idxFoil}.type, 'T foil')
        foilColor = 'm';
        lineWidth = 2;
    elseif contains(foil{idxFoil}.type, 'rudder')
        foilColor = 'y';
        lineWidth = 2;
    else
        foilColor = 'b';
        lineWidth = 2;
    end
    
    % Draw foil
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], foilColor, 'LineWidth', lineWidth);
    
    % Draw force vector
    foilIndividualLoad = foilLoad(eta, nu, foil{idxFoil}, wind, wave, false);
    p1 = eta(1:3) + R*(foilPos);
    p2 = eta(1:3) + R*(foilPos) + scalingForces * 1e-3 * (R * foilIndividualLoad(1:3));
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], foilColor);
    
    % Label foil with number
    text(mean([p1(1) p2(1)]), mean([p1(2) p2(2)]), mean([p1(3) p2(3)]), ...
         num2str(idxFoil), 'FontSize', 12, 'Color', foilColor, 'FontWeight', 'bold');
end

% Draw weight
weightIndividualLoad = weightLoad(eta, false);
p1 = eta(1:3) + R*COGpositionInB;
p2 = eta(1:3) + R*COGpositionInB + scalingForces * 1e-3 * (R * weightIndividualLoad(1:3));
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r', 'LineWidth', 2);
plot3(p1(1), p1(2), p1(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(p1(1), p1(2), p1(3), ' COG', 'FontSize', 10, 'Color', 'r');

% Draw aerodynamic load on superstructure
aeroLoadInB = aerodynamicLoadSuperstructure(eta, nu, wind, false);
p1 = eta(1:3) + R*applicationPointAeroLoadSuperstructure;
p2 = eta(1:3) + R*applicationPointAeroLoadSuperstructure + scalingForces * 1e-3 * (R * aeroLoadInB(1:3));
plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'g', 'LineWidth', 2);
plot3(p1(1), p1(2), p1(3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'g');

% Draw wind direction arrow
windScale = 5;
windDir = wind.direction;
windVec = windScale * [cos(windDir); sin(windDir); 0];
pWind = eta(1:3) + R*[hullLength/2 + 5; 0; -5];
quiver3(pWind(1), pWind(2), pWind(3), windVec(1), windVec(2), windVec(3), ...
        'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(pWind(1) + windVec(1)/2, pWind(2) + windVec(2)/2, pWind(3), ...
     sprintf('Wind %.1f m/s', wind.speedInN), 'FontSize', 10);

% Formatting
axis equal
grid on
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title(sprintf('Catamaran: heel=%.1f°, pitch=%.1f°, heave=%.2fm', ...
              eta(4)*180/pi, eta(5)*180/pi, eta(3)));
set(gca, 'zdir', 'reverse') % more intuitive
set(gca, 'ydir', 'reverse')
view(45, 20);

hold off;

end
