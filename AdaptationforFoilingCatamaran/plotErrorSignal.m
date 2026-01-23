function plotErrorSignal(t, eta, nu, u)
subplot(1,3,1);
hold on; grid on;
%plot(t,eta(:,1));
%plot(t,eta(:,2));
plot(t,eta(:,3));
plot(t,eta(:,4));
plot(t,eta(:,5));
plot(t,eta(:,6));
xlabel('Time $$ (s)$$','Interpreter','Latex');
title('Position ($$m$$ or $$rad$$)','Interpreter','Latex');
%h = legend('$$x(t)$$','$$y(t)$$','$$z(t)$$','$$phi(t)$$','$$theta(t)$$','$$psi(t)$$');
h = legend('$$z(t)$$','$$phi(t)$$','$$theta(t)$$','$$psi(t)$$');
set(h,'Interpreter','latex')

subplot(1,3,2);
hold on; grid on;
%plot(t,nu(:,1));
%plot(t,nu(:,2));
plot(t,nu(:,3));
plot(t,nu(:,4));
plot(t,nu(:,5));
plot(t,nu(:,6));
xlabel('Time $$ (s)$$','Interpreter','Latex');
title('Velocity ($$m/s$$ or $$rad/s$$)','Interpreter','Latex');
%h = legend('$$u(t)$$','$$v(t)$$','$$w(t)$$','$$p(t)$$','$$q(t)$$','$$r(t)$$');
h = legend('$$w(t)$$','$$p(t)$$','$$q(t)$$','$$r(t)$$');
set(h,'Interpreter','latex')

% 5 inputs
subplot(1,3,3);
hold on; grid on;

plot(u(:,1),u(:,2));
plot(u(:,1),u(:,3));
plot(u(:,1),u(:,4));
plot(u(:,1),u(:,5));
plot(u(:,1),u(:,6));
xlabel('Time $$ (sec)$$','Interpreter','Latex');
title('Control Input (rad)','Interpreter','Latex');
%h = legend('$$u(t)$$','$$v(t)$$','$$w(t)$$','$$p(t)$$','$$q(t)$$','$$r(t)$$');
h = legend('L Foil','Starboard T Foil', 'Starboard Rudder', 'Port T Foil', 'Port Rudder');
set(h,'Interpreter','latex')
return