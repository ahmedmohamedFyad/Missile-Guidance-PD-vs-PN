% Proportional Navigation Guidance AHMED Fyad

clear; clc;

N = 3;          % Navigation Constant
dt = 0.01;
t_max = 40;

Mx = 0; My = 0;
Tx = 1000; Ty = 500;
Tvx = 20; Tvy = 0;
Vm = 80;         %constan (m/s)


lambda0 = atan2(Ty-My, Tx-Mx);
Mvx = Vm*cos(lambda0);
Mvy = Vm*sin(lambda0);

Mx_hist = Mx; My_hist = My;
Tx_hist = Tx; Ty_hist = Ty;
lambda_prev = lambda0;

for t = 0:dt:t_max
    Tx = Tx + Tvx*dt;
    Ty = Ty + Tvy*dt;
    dx = Tx-Mx; dy = Ty-My;
    R = sqrt(dx^2+dy^2);
    
    if R < 15
        disp('Target hit!');
        break;
    end
    
    lambda = atan2(dy, dx);
    lambda_dot = (lambda - lambda_prev)/dt;
    lambda_prev = lambda;
    
    Vc = -((dx*(Tvx-Mvx) + dy*(Tvy-Mvy)) / R);
    
    aM = N*Vc*lambda_dot;
    
    
    ax = -aM*sin(lambda);
    ay =  aM*cos(lambda);
    
    Mvx = Mvx + ax*dt; Mvy = Mvy + ay*dt;
    spd = sqrt(Mvx^2 + Mvy^2);
    Mvx = Mvx/spd*Vm; Mvy = Mvy/spd*Vm;
    
    Mx = Mx + Mvx*dt; My = My + Mvy*dt;
    
    Mx_hist = [Mx_hist, Mx]; My_hist = [My_hist, My];
    Tx_hist = [Tx_hist, Tx]; Ty_hist = [Ty_hist, Ty];
end

figure;
plot(Mx_hist, My_hist, 'r-', 'LineWidth', 2);
hold on;
plot(Tx_hist, Ty_hist, 'b--', 'LineWidth', 1.5);
plot(0, 0, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(Tx_hist(end), Ty_hist(end), 'b*', 'MarkerSize', 15);
grid on;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Proportional Navigation Guidance - Ahmed Fyad');
legend('Missile path', 'Target path', 'launch point', 'Target');