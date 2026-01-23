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
foil{1}.positionInB = [4.66;-10*sind(5);-10.62]; 
foil{1}.attitudeInB = [-85;0;-20]*pi/180; 
foil{1}.chord = 13;
foil{1}.span = 22;

%% L foil on the starboard floater
% Terminology: rake (longitudinal variations) and cant (lateral variation)
foil{2}.type = 'starboard L foil horizontal part';
foil{2}.positionInB = [7.75;3.2;2.6-0.935];
%foil{2}.attitudeInB = [-5+20;3.5;0]*pi/180; % Real numbers
foil{2}.attitudeInB = [-5+20;3.3;0]*pi/180; % Paper numbers
foil{2}.chord = 0.4;
foil{2}.span = 2;

foil{3}.type = 'starboard L foil vertical part';
foil{3}.positionInB = [7.75;4.1;0.935];
%foil{3}.attitudeInB = [-5+105;3.5;0]*pi/180; % Real numbers
foil{3}.attitudeInB = [-5+105;3.3;0]*pi/180; % Paper numbers
foil{3}.chord = 0.4;
foil{3}.span = 2;

%% rudder + foil on the starboard
foil{4}.type = 'starboard T foil ';
foil{4}.positionInB = [0.16032;3.744;0.96704 + 0.944];
%foil{4}.attitudeInB = [-5;1;1.4]*pi/180; % Real Numbers
foil{4}.attitudeInB = [-5;0.9;1.7]*pi/180; % Paper Numbers
foil{4}.chord = 0.2;
foil{4}.span = 1.2;

foil{5}.type = 'starboard rudder';
foil{5}.positionInB = [0.16032;3.744;0.96704];
%foil{5}.attitudeInB = [-5+90;0;1.4]*pi/180; % Real Numbers
foil{5}.attitudeInB = [-5+90;0;1.7]*pi/180; % Paper Numbers
foil{5}.chord = 0.2;
foil{5}.span = 2.032;

%% rudder + foil on the port
foil{6}.type = 'port T foil ';
foil{6}.positionInB = [0.16032;-3.744;0.96704 + 0.944];
%foil{6}.attitudeInB = [-5;0.9;1.4]*pi/180; % Real Numbers
foil{6}.attitudeInB = [-5;1.1;1.7]*pi/180; % Paper Numbers
foil{6}.chord = 0.2;
foil{6}.span = 1.2;

foil{7}.type = 'port rudder';
foil{7}.positionInB = [0.16032;-3.744;0.96704];
%foil{7}.attitudeInB = [-5+90;0;1.4]*pi/180; % Real Numbers
foil{7}.attitudeInB = [-5+90;0;1.7]*pi/180; % Paper Numbers
foil{7}.chord = 0.2;
foil{7}.span = 2.032;


end