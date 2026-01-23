function waveForce = waveMechanics(eta, nu, area, H, T, d, L, z, y)
    rho_w = 1025;
    g = 9.81;

    %L = g*T^2/(2*pi)*tanh(2*pi*d/L)
    omega = 2*pi/T;
    a = H/2;
    k = 2*pi/L;
    p = -rho_w*g*z + rho_wg*H/2*cosh(k*(d+z))*cos(k*y-omega*t)/cosh(k*d);

    waveForce = p*A;
end