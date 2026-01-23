function C = coriolisCentripetal(M,nu)
nu2 = nu(4:6);
m = M(1,1);
rG = [M(2,6)/m;-M(3,4)/m;M(1,5)/m];
Ib = M(4:6,4:6);
C = [m*skewSym(nu2) -m*skewSym(nu2)*skewSym(rG);...
    m*skewSym(rG)*skewSym(nu2)  -skewSym(Ib*nu2)];
end