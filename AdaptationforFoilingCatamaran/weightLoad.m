function weightLoadInB = weightLoad(eta,verbose)
% Returns the load expressed in {b}

% Mass propoerties
boatMass = (2.3e3 + 87.5*6) ; %2,332–2,432 kg
COGpositionInB = [5.92896; 0.07904; -3.86752];
g=9.81;


weightForceInB = Rbn(eta).'*[0;0;boatMass*g];
weightLoadInB = [weightForceInB;cross(COGpositionInB,weightForceInB)];
if verbose
    fprintf(1,'w  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | \n',weightLoadInB.'/1000);
end
end