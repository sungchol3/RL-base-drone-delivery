clear all
close all
clc

% Parameters
g = 9.81;
dt = 0.01;
tEnd = 10;
pos0 = [0; 0; 30];
target = [10; 10; 0];

z_hold = 5;
d_gate = 0.1;
closeTol = 0.05;

alpha = 0.2;

gains = [2.5 0.1 1.2;
         2.5 0.1 1.2;
         1.5 0.1 1];

thrustMaxXY = 12;
thrustMaxZ = 15;
intLimZ = 5;

% Pre-allocations
N = round(tEnd/dt)+1;
t = (0:N-1) * dt;
pos = zeros(3, N);
pos(:,1) = pos0;
vel = zeros(3, N);
intErr = zeros(3, 1);
errPrev = zeros(3, 1);
onGround = false;
kLast = N;

d_xy0 = norm(target(1:2)-pos0(1:2));
z_cmd = pos0(3);
state = 1; % 1:glide, 2:hold, 3:drop

% PID Loop
for k = 2:N
    d_xy = norm(target(1:2)-pos(1:2,k-1));

    switch state
        case 1
            z_ref = max(z_hold, pos0(3)*(d_xy/d_xy0));
            if pos(3, k-1) <= z_hold + 0.05
                state = 2;
                intErr(3) = 0;
                vel(3, k-1) = 0;
            end
        case 2
            z_ref = z_hold;
            if d_xy < d_gate
                state = 3;
                intErr(3) = 0;
                vel(1:2, k-1) = 0;
            end
        case 3
            z_ref = 0;
    end
    z_cmd = alpha * z_ref + (1-alpha) * z_cmd;

    err_xy = target(1:2) - pos(1:2, k-1);
    if state == 3
        err_xy = [0; 0];
    end
    err = [err_xy; z_cmd-pos(3, k-1)];

    intErr = intErr + err * dt;
    intErr(3) = max(min(intErr(3), intLimZ), -intLimZ);
    dErr = (err - errPrev)/dt;
    aNet = gains(:,1).*err + gains(:,2).*intErr + gains(:,3).*dErr;
    
    if state == 3
        aNet(1:2) = 0;
    end

    thrust = aNet + [0; 0; -g];
    thrust(1:2) = max(min(thrust(1:2), thrustMaxXY), -thrustMaxXY);
    thrust(3) = max(0, min(thrust(3), thrustMaxZ));

    a = thrust + [0; 0; -g];
    vel(:,k) = vel(:, k-1) + a*dt;
    pos(:,k) = pos(:, k-1) + vel(:,k) * dt;

    if state == 2 && pos(3,k)<z_hold
        pos(3,k) = z_hold;
        if vel(3,k) < 0
            vel(3,k) = 0;
        end
    end

    if pos(3,k) <= 0
        pos(3,k) = 0;
        if ~onGround
            vel(3,k) = 0;
            onGround = true;
        end
    end

    errPrev = err;
end

% Plot
figure('Name', 'PID-Control Flight');
plot3(pos(1,1:kLast), pos(2,1:kLast), pos(3,1:kLast), 'b', 'LineWidth', 1.4);
hold on;
grid on;
plot3(pos0(1), pos0(2), pos0(3), 'gx', 'MarkerSize', 8, 'LineWidth', 2);
plot3(target(1), target(2), target(3), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
title('Flight Trajectory');
view(35, 25);

figure('Name', '3-D Live Animation');
axis equal;
grid on;
hold on;
xlabel X;
ylabel Y;
zlabel Z;
view(35, 25);
plot3(pos0(1), pos0(2), pos0(3), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
plot3(target(1), target(2), target(3), 'ro', 'MarkerSize', 9, 'LineWidth', 2);
h = animatedline('LineWidth', 1.4);
for k = 1:kLast
    addpoints(h, pos(1,k), pos(2,k), pos(3,k));
    drawnow;
    pause(dt)
end