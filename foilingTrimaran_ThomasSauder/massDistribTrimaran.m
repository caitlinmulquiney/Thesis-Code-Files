function M = massDistribTrimaran(verbose)

% Rough estimation of the inertia properties of a sailing trimaran. See attached
% notes for details and assumptions. Reference point is below mast, at
% deck level. 

mf=1.85e3; % mass of a floater
mc=3.3e3; % mass of the central hull
mm=1400; % mass of the mast and sails
mt=1.75e3; % mass of the lateral beams
mco=2.5e3; % mass of the cockpit (includes the cockpit + equipent, main sail traveller, etc...)

lf=27.6; %length of floaters (we neglect the bow part ahead of the forestay to fit our geometric assumption)
lc=27; % length of central hull (we neglect the bow part ahead of the forestay to fit our geometric assumption)
df=10.5; % distance between central hull and floaters
lm=35; % mast height, from wikipedia
dt=6; % distance between mast and transvers beams
dco=7.5; % distance between mast and cockpit (includes the cockpit + equipent, main sail traveller, etc...)

% Total mass
mtot = 2*mf + mc + mm + 2*mt + mco;
% Position of COG (see assumptions)
xG = -mco*dco/mtot;
zG = mm*lm/(2*mtot);
% Moment of inertia in roll
Ixx = 2*mf*df^2 + 1/3*mm*lm^2 + 2/3*mt*df^2;    
rxx = sqrt(Ixx/mtot);
% Moment of inertia in pitch
Iyy = 1/6*mf*lf^2 + 1/12*mc*lc^2 + 1/3*mm*lm^2 + 2*mt*dt^2 + mco*dco^2;
ryy = sqrt(Iyy/mtot);
% Moment of inertia in yaw
Izz = 2*mf*df^2 + 1/6*mf*lf^2 + 1/12*mc*lc^2 + 2*mt*dt^2 + 1/12*mt*df^2 + mco*dco^2;
rzz = sqrt(Izz/mtot);

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

end

