function M = massDistribTrimaran(verbose)
% Rough estimation of the inertia properties of a sailing trimaran. See attached
% notes for details and assumptions. Reference point is below mast, at
% deck level. 

% Total mass
mtot = (2.3e3 + 87.5*6 ) ;
% Position of COG (see assumptions)
xG = 5.92896;
yG = 0.07904;
zG = -3.86752;
% Moment of inertia in roll
Ixx_cog = 129000;
rxx = sqrt(Ixx_cog/mtot);
% Moment of inertia in pitch
Iyy_cog = 143000;
ryy = sqrt(Iyy_cog/mtot);
% Moment of inertia in yaw
Izz_cog = 52000;
rzz = sqrt(Izz_cog/mtot);

Ixx = Ixx_cog + mtot * (yG^2 + zG^2);
Iyy = Iyy_cog + mtot * (xG^2 + zG^2);
Izz = Izz_cog + mtot * (xG^2 + yG^2);
% 
% Iyy = 299686;
% Izz = 161775;
% Ixx = 175778;
% if verbose
% disp('------------------------')
% disp(['Total mass = ' num2str(mtot/1e3,3) ' t'] );
% disp('------------------------')
% disp('COG in {b}')
% disp(['OG.b1 = ' num2str(xG,3) ' m'] );
% disp(['OG.b3 = ' num2str(-zG,3) ' m'] );
% disp('------------------------')
% disp('Roll')
% disp(['I44 = ' num2str(Ixx/1e3,3) ' t.m^2'] )
% disp(['r44 = ' num2str(rxx,3) ' m'] );
% disp([num2str(2*mf*df^2/Ixx*100,2) '% comes from the floaters'])
% disp([num2str(1/3*mm*lm^2/Ixx*100,2) '% comes from the mast'])
% disp([num2str(2/3*mt*df^2/Ixx*100,2) '% comes from the beams'])
% disp('------------------------')
% disp('Pitch')
% disp(['I55 = ' num2str(Iyy/1e3,3) ' t.m^2'] )
% disp(['r55 = ' num2str(ryy,3) ' m'] );
% disp([num2str(1/6*mf*lf^2/Iyy*100,2) '% comes from the floaters'])
% disp([num2str(1/12*mc*lc^2/Iyy*100,2) '% comes from the central hull'])
% disp([num2str(1/3*mm*lm^2/Iyy*100,2) '% comes from the mast'])
% disp([num2str(2*mt*dt^2/Iyy*100,2) '% comes from the beams'])
% disp([num2str(mco*dco^2/Iyy*100,2) '% comes from the cockpit and equipment'])
% disp('------------------------')
% disp('Yaw')
% disp(['I66 = ' num2str(Izz/1e3,3) ' t.m^2'] )
% disp(['r66 = ' num2str(rzz,3) ' m'] );
% disp([num2str((2*mf*df^2 + 1/6*mf*lf^2)/Izz*100,2) '% comes from the floaters'])
% disp([num2str(1/12*mc*lc^2/Izz*100,2) '% comes from the central hull'])
% disp([num2str((2*mt*dt^2 + 1/12*mt*df^2)/Izz*100,2) '% comes from the beams'])
% disp([num2str((mco*dco^2)/Izz*100,2) '% comes from the cockpit and equipment'])
% disp('------------------------')
% end

% Fill in the mass matrix
M = zeros(6,6);
M(1,1) = mtot;
M(2,2) = mtot; 
M(3,3) = mtot;
M(4,4) = Ixx; 
M(5,5) = Iyy;
M(6,6) = Izz;
zG = -zG; % Express in {b}
M(1,5) = mtot*zG;   M(5,1)=M(1,5);
M(2,4) = -mtot*zG;  M(4,2)=M(2,4);
M(2,6) = mtot*xG;   M(6,2)=M(2,6);
M(3,5) = -mtot*xG;  M(5,3)=M(3,5);
M(1,6) = -mtot*yG;   M(6,1) = M(1,6);
M(3,4) = mtot*yG;    M(4,3) = M(3,4);

end



