function J = configurationMatrix(foil,eta,nu,wind,wave)

du = .1*pi/180;
foilList = [4 5 6 8 9];

J = zeros(6,5);
for idx = 1:length(foilList)
    foil_ = foil{foilList(idx)};
    if foilList(idx) == 5 || foilList(idx) == 9
        dFoilAngle = [0;0;du]; %for this foil (rudder), the control input is the "yaw" of the foil
    else
        dFoilAngle = [0;du;0]; %for the other ones, it is the "pitch"
    end
    foil_.attitudeInB = foil{foilList(idx)}.attitudeInB + dFoilAngle;
    dLoadPlus = foilLoad(eta,nu,foil_,wind,wave,false);
    foil_.attitudeInB = foil{foilList(idx)}.attitudeInB - dFoilAngle;
    dLoadMinus = foilLoad(eta,nu,foil_,wind,wave,false);
    J(:,idx) = (dLoadPlus-dLoadMinus)/(2*du);
end

end