function weightLoadInB = weightLoad(eta,verbose)
% Returns the load expressed in {b}

% Mass propoerties
boatMass = 14.4e3   ;
COGpositionInB = [-1.3; 0; -2.4];
g=9.81;


weightForceInB = Rbn(eta).'*[0;0;boatMass*g];
weightLoadInB = [weightForceInB;cross(COGpositionInB,weightForceInB)];
% if verbose
%     fprintf(1,'w  |                                                | %+10.1f %+10.1f %+10.1f | %+10.1f %+10.1f %+10.1f | \n',weightLoadInB.'/1000);
% end
end