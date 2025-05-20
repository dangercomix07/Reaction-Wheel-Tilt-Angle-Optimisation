%% Reaction Wheel Tilt Angle Optimisation

% Study to find optimal tilt angle for a 
% 4-wheel pyramidal reaction wheel system

% Author: Ameya Marakarkandy
% ameya.marakarkandy@gmail.com

% MIT License
% 
% Copyright (c) 2025 Ameya Marakarkandy
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%% ORBITAL PARAMETERS

orb.Mu = 3.986e14;  % Standard Gravitational Parameter of Earth
orb.Re = 6371e3;    % Radius of Earth
orb.alt = 500e3;    % Altitude

orb.a = orb.Re + orb.alt;         % Semi-major axis
orb.e = 0;                        % Eccentricity
orb.i = deg2rad(90);              % Inclination
orb.RAAN = 0;                     % Right Accension of Ascending Node
orb.w = deg2rad(90);              % Argument of Periapsis
orb.ta = deg2rad(0);              % True Anomaly
orb.n = sqrt(orb.Mu/(orb.a)^3);   % Mean Motion
orb.T = 2*pi/orb.n;
orb.omegaON = [0;-orb.n;0];

start_time = juliandate(2025, 1, 1, 12, 0, 0);

%% SATELLITE PARAMETERS

% Inertia Tensor (B-frame)
sat.Jx = 10;
sat.Jy = 10;
sat.Jz = 12;

sat.Jxz = 0;
sat.Jxy = 0;
sat.Jyz = 0;
sat.J = [sat.Jx,-sat.Jxy,-sat.Jxz;...
        -sat.Jxy,sat.Jy,-sat.Jyz;...
        -sat.Jxz,-sat.Jyz,sat.Jz];

% omega B wrt O in B-frame
sat.p = 0;
sat.q = 0;
sat.r = 0;
sat.omegaBO = [sat.p;sat.q;sat.r];

% omega B wrt N in B-frame
sat.w1 = 0 + sat.p;
sat.w2 = -orb.n + sat.q;
sat.w3 = 0 + sat.r;
sat.omegaBN = [sat.w1;sat.w2;sat.w3];

initAngle = deg2rad(0);
initAxis = [0;1;0];
sat.quat = [cos(initAngle/2);sin(initAngle/2)*initAxis];

%% REACTION WHEELS

% Reaction Wheel Inertia about axis
Js = 0.001;
omega0 = 0; % Initial wheel speed

% Tilt Angle (beta)
beta_range = 10:1:70;  % Tilt angle range (degrees)
num_angles = length(beta_range);
enorm = zeros(1,num_angles);
hnorm = zeros(1,num_angles);

function A = RWSConfig(B)
    A = [sin(B),0,-sin(B),0;
        0,sin(B),0,-sin(B);
        cos(B),cos(B),cos(B),cos(B)];
end

%% ATTITUDE CONTROLLER 

Kp = diag([10;10;12]);
Kd = diag([20;20;24]);

%% SIMULATION 

for i = 1:num_angles
    beta = deg2rad(beta_range(i));
    A = RWSConfig(beta);

    assignin('base','A',A);
    out = sim("AttitudeControlSim.slx");
    enorm(i) = out.error_norm.Data(end);
    hnorm(i) = out.angmomentum_norm.Data(end);
end

%% VISUALISATION

% Apply smoothing
enorm = smooth(enorm, 0.2, 'loess');  % 0.2 = smoothing parameter
hnorm = smooth(hnorm, 0.2, 'loess');

% Plotting the results
figure;
subplot(2,1,1);
plot(beta_range, enorm, '-', 'LineWidth', 1.5);
xlabel('Tilt Angle (deg)');
ylabel('Quaternion Error Norm');
title('Quaternion Error Norm vs. Tilt Angle');
grid on;

subplot(2,1,2);
plot(beta_range, hnorm, '-', 'LineWidth', 1.5);
xlabel('Tilt Angle (deg)');
ylabel('Angular Momentum Norm');
title('Angular Momentum Norm vs. Tilt Angle');
grid on;

%% OPTIMISATION

% Normalising the evaluation metrics
enorm_norm = (enorm - min(enorm)) / (max(enorm) - min(enorm));
hnorm_norm = (hnorm - min(hnorm)) / (max(hnorm) - min(hnorm));

w1 = 0.7;  % weight for pointing accuracy (enorm)
w2 = 0.3;  % weight for control effort (hnorm)

% Compute combined loss
loss = w1 * enorm_norm + w2 * hnorm_norm;

[~, idx_min] = min(loss);         % index of optimal beta
beta_opt = beta_range(idx_min);  % optimal tilt angle

figure;
plot(beta_range, loss, '-k', 'LineWidth', 1.5);
hold on;
plot(beta_opt, loss(idx_min), 'ro', 'MarkerSize', 8, 'DisplayName', 'Optimum');
xlabel('Tilt Angle (deg)');
ylabel('Combined Loss');
title('Optimization of Tilt Angle');
legend('Combined Loss', 'Optimal \beta');
grid on;

fprintf('Optimal tilt angle (beta): %.2f deg\n', beta_opt);
fprintf('Minimum loss: %.4e\n', loss(idx_min));
fprintf('Error norm at optimum: %.4e\n', enorm(idx_min));
fprintf('Momentum norm at optimum: %.4e\n', hnorm(idx_min));


