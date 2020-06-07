% Author : Ghananeel Rotithor

% Create state vector and P matrix and time vector
iter = 1000;  dt = 0.01; t = 0:dt:(iter-1)*dt;
x = zeros(2,1000); P = zeros(2,2,1000);

% Set initial conditions
x(:,1) = [5,0]; P(:,:,1) = ones(2,2);

% Create State Transition and Cost Matrices
A = [0,1;2,3];  B = [0;1]; Q = ones(2,2); R = 1;

for i = 2:size(x,2)
    % Propagate P matrix using CRE 
    P_dot =  P(:,:,i-1)*A + A'*P(:,:,i-1) - P(:,:,i-1)*(B*B')*P(:,:,i-1) + Q;
    P(:,:,i) = P(:,:,i-1) + P_dot*dt;
    
    % Compute Optimal Gain using P
    K(i-1,:) = B'*P(:,:,i);
    
    % Compute Control Input
    u(i-1) = -K(i-1,:)*x(:,i-1);
    
    % Apply Control and Propagate Dynamical System
    x_dot = A*x(:,i-1) + B * u(i-1);
    x(:,i) = x(:,i-1) + x_dot * dt;
end

subplot(221)
plot(t,x','LineWidth',2);
xlabel('Time (s)','interpreter','latex','FontSize',20);
ylabel('State','interpreter','latex','FontSize',20);
legend('$\xi$','$\dot{\xi}$','interpreter','latex','FontSize',20);

subplot(222)
plot(t(1:end-1),u,'LineWidth',2);
xlabel('Time (s)','interpreter','latex','FontSize',20);
ylabel('Control Effort','interpreter','latex','FontSize',20);

subplot(223)
plot(t(1:end-1),K,'LineWidth',2);
xlabel('Time (s)','interpreter','latex','FontSize',20);
ylabel('Control Gain','interpreter','latex','FontSize',20);
legend('$K_{1}$','$K_{2}$','interpreter','latex','FontSize',20);

subplot(224)
plot(t,reshape(P(1,1,:),1,size(P,3)),'LineWidth',2); hold on
plot(t,reshape(P(1,2,:),1,size(P,3)),'--','LineWidth',2);
plot(t,reshape(P(2,1,:),1,size(P,3)),':','LineWidth',2);
plot(t,reshape(P(2,2,:),1,size(P,3)),'LineWidth',2);
xlabel('Time (s)','interpreter','latex','FontSize',20);
ylabel('$P$ Matrix','interpreter','latex','FontSize',20);
legend('$P_{11}$','$P_{12}$','$P_{21}$','$P_{22}$',...
        'interpreter','latex','FontSize',20);

