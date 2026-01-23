function foil = loadFoilDescription
% Returns a structure containing the description of the foils: sail,
% rudders, T-foils, and the L-foil.
% NB: only the foils of interest when the wind comes from port are described
% And apparently we should also add the central rudder
% 
% A coordinate system {f} is attached to each foil
% Origin F: center of the foil (mid-chord, mid-span)
% f1 is along the chord
% f2 is along the span
% f3 is orthogonal to the foil

% In the structure below, one gives the position of F, the chord and span [m], 
% and the attitude of {f} in {b}, which defines, among other the angle of
% attack

% NB: these values are not exactly consistent with the ones for Macif
% (I played a bit with them to get reasonably close to static equiilibrium)

% Dihedron causes starboard floater to be .92 (10.5*sind(5)) m above
% central floater, and 5deg tilted


%% sail
foil{1}.type = 'sail';
foil{1}.positionInB = [-1;-20*sind(5);-20]; 
foil{1}.attitudeInB = [-95;0;-20]*pi/180; 
foil{1}.chord = 13;
foil{1}.span = 25;

%% L foil on the starboard floater
% Terminology: rake (longitudinal variations) and cant (lateral variation)
foil{2}.type = 'starboard floater L foil horizontal part';
foil{2}.positionInB = [4;8.5;2-.92];
foil{2}.attitudeInB = [-5+20;4.5;0]*pi/180; 
foil{2}.chord = 0.6;
foil{2}.span = 2;

foil{3}.type = 'starboard floater L foil vertical part';
foil{3}.positionInB = [4;9.75;1.5-.92];
foil{3}.attitudeInB = [-5+105;4.5;0]*pi/180; 
foil{3}.chord = 0.6;
foil{3}.span = 2;

%% rudder + foil on the starboard floater
foil{4}.type = 'starboard floater T foil ';
foil{4}.positionInB = [-12.5;10.5;1.2-.92];
foil{4}.attitudeInB = [-5;0;0]*pi/180; 
foil{4}.chord = 0.5;
foil{4}.span = 0.5;

foil{5}.type = 'starboard floater rudder';
foil{5}.positionInB = [-12.5;10.5;0.6-.92];
foil{5}.attitudeInB = [-5+90;0;0]*pi/180; 
foil{5}.chord = 0.5;
foil{5}.span = 1.2;

%% centerboard + T-foil on central hull
foil{6}.type = 'centerboard T foil part';
foil{6}.positionInB = [-1;0;4.2];
foil{6}.attitudeInB = [0;0;0]*pi/180; 
foil{6}.chord = 0.5;
foil{6}.span = 1.1; %reduced from 1.3 due to presence of vertical part

foil{7}.type = 'centerboard vertical part';
foil{7}.positionInB = [0;0;2.1];
foil{7}.attitudeInB = [90;-13;0]*pi/180; 
foil{7}.chord = 0.5;
foil{7}.span = 4.2;

%% rudder + foil on the central hull
foil{8}.type = 'centerhull rudder T foil';
foil{8}.positionInB = [-11;0;2.1];
foil{8}.attitudeInB = [0;0;0]*pi/180; 
foil{8}.chord = 0.5;
foil{8}.span = 0.5;

foil{9}.type = 'centerhull rudder';
foil{9}.positionInB = [-11;0;1.05];
foil{9}.attitudeInB = [90;0;0]*pi/180; 
foil{9}.chord = 0.5;
foil{9}.span = 2.1;


end