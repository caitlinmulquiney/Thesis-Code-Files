function foilLoadInB = foilLoad(eta,nu,foil,wind,wave,verbose)  %#ok<INUSL>

% Inputs: 
% - eta and nu (following Fosen notations)
% - foil description (one of the cell in the cell array returned by loadFoilDescription 
% - wind and wave descritpion (for now only wind is considered)
% - a boolean "verbose" to print details or not

% Densities [kg/m^3]
rho_air=1;
rho_water=1025;

% First of all, check whether we are in air, in water, or partially
% submerged
foilEnd1Position = eta(1:3) + Rbn(eta)*(foil.positionInB + Rbn([zeros(3,1);foil.attitudeInB])*[0;-foil.span/2;0]);
foilEnd2Position = eta(1:3) + Rbn(eta)*(foil.positionInB + Rbn([zeros(3,1);foil.attitudeInB])*[0;foil.span/2;0]);
Dplus = max([foilEnd1Position(3) foilEnd2Position(3)]);
Dminus = min([foilEnd1Position(3) foilEnd2Position(3)]);
if sign(Dplus)~=sign(Dminus)
    foilStatus = 'partiallySubmerged';
    sigma = Dplus/(Dplus-Dminus);
else
    if Dminus>0
        foilStatus = 'inWater';
        sigma = 1;
    else
        foilStatus = 'inAir';
    end
end

% Compute relative velocity of the foil wrt to the fluid, in {b}
foilLinearVelocityInB = nu(1:3)+cross(nu(4:6),foil.positionInB);
windVelocityInN = [wind.speedInN*cos(wind.direction);wind.speedInN*sin(wind.direction);0];
waveVelocityInN = [0;0;0];
if strcmp(foilStatus,'inAir')
    flowLinearVelocityInB = Rbn(eta).'*windVelocityInN;
else
    flowLinearVelocityInB = Rbn(eta).'*waveVelocityInN;
end
relativeLinearVelocityInB = foilLinearVelocityInB - flowLinearVelocityInB;


% Transport this into {f}, the foil-linked coordinate system, that is such
% that:
% f1 is along the chord
% f2 is along the span
% f3 is orthogonal to the foil
relativeLinearVelocityInF = Rbn([zeros(3,1);foil.attitudeInB]).' * relativeLinearVelocityInB;

% Relative speed and angle of attack (ingredients to compute the drag and lift)
foilRelativeSpeed = norm(relativeLinearVelocityInF);
foilAngleOfAttack = atan2(dot([0,0,1],relativeLinearVelocityInF),dot([1,0,0],relativeLinearVelocityInF));

% Formulas to compute lift and drag coefficients (very simple, based on
% figures found in Faltinsen, High-speed marine vehicles book, Figures 2.17 and 2.18)
liftCoeff=min([1.5 1.5*foilAngleOfAttack/(15*pi/180)]); %linearly increasing from 0 to 1.5 between 0 and 15 degrees, and saturates at 1.5
dragCoeff=0.01 +0.015*(foilAngleOfAttack/(20*pi/180))^2; %bucket-shaped  with Cd=0.025 at 20 deg

% Lift (orthogonal to the _flow_, not foil) and drag (along the flow)
if strcmp(foilStatus,'inAir')
    lift = 1/2 * rho_air * liftCoeff * foilRelativeSpeed^2 * foil.chord * foil.span;
    drag = 1/2 * rho_air * dragCoeff * foilRelativeSpeed^2 * foil.chord * foil.span;
else
    lift = 1/2 * sigma * rho_water * liftCoeff * foilRelativeSpeed^2 * foil.chord * foil.span;
    drag = 1/2 * sigma * rho_water * dragCoeff * foilRelativeSpeed^2 * foil.chord * foil.span;
end

% Convert the drag and lift to a force/moment expressed in {b}
foilForceInF = Rbn([zeros(3,1);0;-foilAngleOfAttack;0])*[-drag;0;-lift];
foilForceInB = Rbn([zeros(3,1);foil.attitudeInB]) * foilForceInF;
foilLoadInB = [foilForceInB; cross(foil.positionInB,foilForceInB)];

% Display if asked to
% if verbose
%     fprintf(1,'f  |  %10.1f %10.1f | %10.1f %10.1f | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | %s (%s)',...
%         foilRelativeSpeed,abs(foilAngleOfAttack)*180/pi,abs(lift)/1e3,abs(drag)/1e3,foilLoadInB.'/1000,foil.type,foilStatus)
%     fprintf('\n')
end

