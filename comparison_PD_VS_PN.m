% PD vs PN Guidance — Animated Comparison
% Ahmed Fyad

clear; clc; close all;

dt = 0.02;       % time step 
t_max = 30;
N = 3;           % Navigation Constant
Vm = 80;         % Missile constant speed (m/s)

% Predict Intercept Point (for initial missile heading) 
Mx0 = 0; My0 = 0;
Tx0 = 1000; Ty0 = 500;
Tvx = 20; Tvy = 0;

rel_x = Tx0 - Mx0; rel_y = Ty0 - My0;
a = Tvx^2 + Tvy^2 - Vm^2;
b = 2*(rel_x*Tvx + rel_y*Tvy);
c = rel_x^2 + rel_y^2;
disc = b^2 - 4*a*c;
t1 = (-b + sqrt(disc))/(2*a);
t2 = (-b - sqrt(disc))/(2*a);
t_int = max([t1, t2] .* ([t1,t2] > 0));
ix = Tx0 + Tvx*t_int;
iy = Ty0 + Tvy*t_int;
lam0 = atan2(iy - My0, ix - Mx0);

% PN Guidance Simulation 
Mx = Mx0; My = My0;
Mvx = Vm*cos(lam0); Mvy = Vm*sin(lam0);
Tx = Tx0; Ty = Ty0;
lambda_prev = atan2(Ty-My, Tx-Mx);

Mx_pn = Mx; My_pn = My;
Tx_hist = Tx; Ty_hist = Ty;

for t = 0:dt:t_max
    Tx = Tx + Tvx*dt;
    Ty = Ty + Tvy*dt;
    dx = Tx - Mx; dy = Ty - My;
    R = sqrt(dx^2 + dy^2);
    if R < 15
        disp('PN: Target hit!');
        break;
    end
    lam = atan2(dy, dx);
    lam_dot = (lam - lambda_prev)/dt;
    Vc = -((dx*(Tvx-Mvx) + dy*(Tvy-Mvy))/R);   % closing velocity 
    lambda_prev = lam;
    aM = N*Vc*lam_dot;
    ax = aM*cos(lam); ay = aM*sin(lam);
    Mvx = Mvx + ax*dt; Mvy = Mvy + ay*dt;
    spd = sqrt(Mvx^2+Mvy^2);
    Mvx = Mvx/spd*Vm; Mvy = Mvy/spd*Vm; 
    Mx = Mx + Mvx*dt; My = My + Mvy*dt;
    Mx_pn = [Mx_pn, Mx]; My_pn = [My_pn, My];
    Tx_hist = [Tx_hist, Tx]; Ty_hist = [Ty_hist, Ty];
end

% PD Guidance Simulation 
Kp = 2; Kd = 5;
Mx = Mx0; My = My0;
Mvx = 0; Mvy = 0;
Tx = Tx0; Ty = Ty0;
ex_prev = Tx-Mx; ey_prev = Ty-My;

Mx_pd = Mx; My_pd = My;

for t = 0:dt:t_max
    Tx = Tx + Tvx*dt;
    Ty = Ty + Tvy*dt;
    ex = Tx-Mx; ey = Ty-My;
    R = sqrt(ex^2+ey^2);
    if R < 15
        disp('PD: Target hit!');
        break;
    end
    ax = Kp*ex + Kd*(ex-ex_prev)/dt;
    ay = Kp*ey + Kd*(ey-ey_prev)/dt;
    ex_prev = ex; ey_prev = ey;
    Mvx = Mvx + ax*dt; Mvy = Mvy + ay*dt;
    Mx = Mx + Mvx*dt; My = My + Mvy*dt;
    Mx_pd = [Mx_pd, Mx]; My_pd = [My_pd, My];
end

%Animation 
figure('Color','k');
set(gca,'Color','k','XColor','w','YColor','w');
hold on; grid on;
xlim([-50, 1500]); ylim([-50, 600]);
xlabel('X Position (m)','Color','w');
ylabel('Y Position (m)','Color','w');
title('PD vs PN Guidance — Live Animation — Ahmed Fyad','Color','w');

plot(0,0,'go','MarkerSize',12,'MarkerFaceColor','g'); % launch
h_target = plot(Tx_hist(1), Ty_hist(1), 'w^', 'MarkerSize', 12, 'MarkerFaceColor','w');
h_target_trail = plot(Tx_hist(1), Ty_hist(1), 'w--', 'LineWidth', 1);
h_pn = plot(Mx_pn(1), My_pn(1), 'r-', 'LineWidth', 2);
h_pn_dot = plot(Mx_pn(1), My_pn(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');
h_pd = plot(Mx_pd(1), My_pd(1), 'b-', 'LineWidth', 2);
h_pd_dot = plot(Mx_pd(1), My_pd(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor','b');
legend([h_pd, h_pn, h_target], {'PD Controller','PN Guidance','Target'}, 'TextColor','w');

n_steps = max(length(Mx_pn), length(Mx_pd));
for k = 2:3:n_steps
    if k <= length(Mx_pn)
        set(h_pn, 'XData', Mx_pn(1:k), 'YData', My_pn(1:k));
        set(h_pn_dot, 'XData', Mx_pn(k), 'YData', My_pn(k));
    end
    if k <= length(Mx_pd)
        set(h_pd, 'XData', Mx_pd(1:k), 'YData', My_pd(1:k));
        set(h_pd_dot, 'XData', Mx_pd(k), 'YData', My_pd(k));
    end
    if k <= length(Tx_hist)
        set(h_target, 'XData', Tx_hist(k), 'YData', Ty_hist(k));
        set(h_target_trail, 'XData', Tx_hist(1:k), 'YData', Ty_hist(1:k));
    end
    drawnow;
end