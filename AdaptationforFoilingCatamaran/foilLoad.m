function foilLoadInB = foilLoad(eta,nu,foil,wind,wave,verbose)  %#ok<INUSL>

% Inputs: 
% - eta and nu (following Fosen notations)
% - foil description (one of the cell in the cell array returned by loadFoilDescription 
% - wind and wave descritpion (for now only wind is considered)
% - a boolean "verbose" to print details or not

% Densities [kg/m^3]
rho_air=1.225;
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
% Formulas to compute lift and drag coefficients
aoa_deg = foilAngleOfAttack * 180/pi;

if strcmp(foil.type, 'sail')
    n_sections = 10;  % spanwise integration points
    s = linspace(0, 1, n_sections);  % normalised span position
    local_aoa_deg = aoa_deg - foil.twist * 180/pi .* s.^foil.twist_exponent;
    lift_total = 0;
    drag_total = 0;
    
    ds = foil.span / (n_sections - 1);
    
    for i = 1:n_sections
        aoa_i = local_aoa_deg(i);
        beta_deg = foil.beta * 180/pi;
        
        liftCoeff_i = (0.1*abs(beta_deg) + 0.1*abs(aoa_i))/2;
        dragCoeff_i = (0.0007+0.00012*abs(beta_deg))*abs(aoa_i)^2;
    
        if abs(aoa_deg) > (14-0.4*abs(beta_deg))
           liftCoeff_i = 5 - 0.3*abs(aoa_i);
        end  

        if aoa_i < 0
            liftCoeff_i = -liftCoeff_i;
        end
        
        % Section lift and drag
        dL = 0.5 * rho_air * liftCoeff_i * foilRelativeSpeed^2 * foil.chord * ds;
        dD = 0.5 * rho_air * dragCoeff_i * foilRelativeSpeed^2 * foil.chord * ds;
        
        lift_total = lift_total + dL;
        drag_total = drag_total + dD;
    end
    
    lift = lift_total;
    drag = drag_total;
else
    liftCoeff = 0.09*(aoa_deg) - 0.0002;
    dragCoeff = 6E-05*(aoa_deg)^2 + 5E-07*abs(aoa_deg) + 0.0051;
    if (abs(aoa_deg) > 15)
        liftCoeff = max(0,5-0.25*abs(aoa_deg));
    end

    % Free Surface Effect
    if strcmp(foil.type, "starboard T foil") || strcmp(foil.type, "port T foil") || strcmp(foil.type, "starboard L foil horizontal part")
        center_pos =  eta(1:3) + Rbn(eta)*foil.positionInB;
        height_chord_ratio = center_pos(3)/foil.chord;
        Lift_fs = min(1.0,0.5*log10(height_chord_ratio+0.1)+0.75);
        Drag_fs = min(1.0,0.5*log10(height_chord_ratio+0.1)+0.9);
    else
        Lift_fs = 1.0;
        Drag_fs = 1.0;
    end

    lift = 1/2 * sigma * rho_water * liftCoeff * foilRelativeSpeed^2 * foil.chord * foil.span * Lift_fs;
    drag = 1/2 * sigma * rho_water * dragCoeff * foilRelativeSpeed^2 * foil.chord * foil.span * Drag_fs;
end

% Convert the drag and lift to a force/moment expressed in {b}
foilForceInF = Rbn([zeros(3,1);0;-foilAngleOfAttack;0])*[-drag;0;-lift];
foilForceInB = Rbn([zeros(3,1);foil.attitudeInB]) * foilForceInF;
if strcmp(foil.type, 'sail')
    coe_height = foil.span - foil.span/foil.twist_exponent;
    if foil.twist_exponent < 0
        coe_height = foil.twist_exponent;
    end
    foilLoadInB = [foilForceInB; cross([foil.positionInB(1); foil.positionInB(2); -coe_height], foilForceInB)];

else 
    foilLoadInB = [foilForceInB; cross(foil.positionInB,foilForceInB)];
end
% Display if asked to
if verbose
    fprintf(1,'f  |  %10.1f %10.1f | %10.1f %10.1f | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | %s (%s)',...
        foilRelativeSpeed,abs(foilAngleOfAttack)*180/pi,abs(lift)/1e3,abs(drag)/1e3,foilLoadInB.'/1000,foil.type,foilStatus)
    fprintf('\n')
end

