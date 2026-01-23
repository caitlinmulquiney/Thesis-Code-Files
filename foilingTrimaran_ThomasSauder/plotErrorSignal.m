function plotErrorSignal(t, eta, nu)
subplot(1,2,1);
hold on; grid on;
%plot(t,eta(:,1));
%plot(t,eta(:,2));
plot(t,eta(:,3));
plot(t,eta(:,4));
plot(t,eta(:,5));
plot(t,eta(:,6));
xlabel('Time $$ (sec)$$','Interpreter','Latex');
title('Position $$ (m/rad)$$','Interpreter','Latex');
%h = legend('$$x(t)$$','$$y(t)$$','$$z(t)$$','$$phi(t)$$','$$theta(t)$$','$$psi(t)$$');
h = legend('$$z(t)$$','$$phi(t)$$','$$theta(t)$$','$$psi(t)$$');
set(h,'Interpreter','latex')

subplot(1,2,2);
hold on; grid on;
%plot(t,nu(:,1));
%plot(t,nu(:,2));
plot(t,nu(:,3));
plot(t,nu(:,4));
plot(t,nu(:,5));
plot(t,nu(:,6));
xlabel('Time $$ (sec)$$','Interpreter','Latex');
title('Velocity $$ (m/rad)$$','Interpreter','Latex');
%h = legend('$$u(t)$$','$$v(t)$$','$$w(t)$$','$$p(t)$$','$$q(t)$$','$$r(t)$$');
h = legend('$$w(t)$$','$$p(t)$$','$$q(t)$$','$$r(t)$$');
set(h,'Interpreter','latex')

return