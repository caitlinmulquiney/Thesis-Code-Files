function J = Jbn(eta)
J = [Rbn(eta) zeros(3,3);
    zeros(3,3) Tbn(eta)];
end