% System Parameters
R = 1.0;       % Armature resistance (Ohms)
L = 0.5;       % Armature inductance (H)
J = 0.01;      % Rotor inertia (kg.m^2)
B = 0.1;       % Viscous friction (N.m/rad/s)
Ke = 0.01;     % Back EMF constant (V/rad/s)
Kt = 0.01;     % Motor torque constant (N.m/A)

% Open-Loop Transfer Function
num = [Kt];
den = [J*L, J*R + B*L, B*R + Ke*Kt];
G = tf(num, den);

% Target Specifications
desired_overshoot = 8; % Desired overshoot in percentage
desired_settling_time = 0.85; % Desired settling time in seconds

% Objective function for tuning
% The function minimizes the error between the system response and the desired criteria
objective = @(x) evaluate_system(x, G, desired_overshoot, desired_settling_time);

% Initial guess for the Lead-Lag compensator parameters
initial_guess = [0.15, 0.03, 2.0, 0.1, 100]; % [T1, T2, T3, T4, K]

% Bounds for the parameters to avoid unrealistic values
lb = [0.01, 0.01, 0.5, 0.01, 50];  % Lower bounds for [T1, T2, T3, T4, K]
ub = [0.5, 0.1, 10, 0.5, 500];      % Upper bounds for [T1, T2, T3, T4, K]

% Use 'fmincon' to find the optimal parameters
options = optimset('Display', 'iter', 'Algorithm', 'interior-point');
optimal_params = fmincon(objective, initial_guess, [], [], [], [], lb, ub, [], options);

% Display the optimal parameters
disp('Optimal Lead-Lag Compensator Parameters:');
disp(['T1 = ', num2str(optimal_params(1))]);
disp(['T2 = ', num2str(optimal_params(2))]);
disp(['T3 = ', num2str(optimal_params(3))]);
disp(['T4 = ', num2str(optimal_params(4))]);
disp(['K = ', num2str(optimal_params(5))]);

% Final system evaluation with optimal parameters
T1_opt = optimal_params(1);
T2_opt = optimal_params(2);
T3_opt = optimal_params(3);
T4_opt = optimal_params(4);
K_opt = optimal_params(5);

% Lead-Lag Compensator with optimal parameters
D_opt = K_opt * tf([T1_opt, 1], [T2_opt, 1]) * tf([T3_opt, 1], [T4_opt, 1]);

% Compensated System
Gc_opt = series(D_opt, G);
Closed_Loop_opt = feedback(Gc_opt, 1);

% Step Response
figure;
step(Closed_Loop_opt);
title('Optimized Lead-Lag Compensated System Step Response');
grid on;

% Step Info
info_opt = stepinfo(Closed_Loop_opt);
disp('Optimized Lead-Lag Compensated System Characteristics:');
disp(info_opt);

% Objective Function: Evaluate the system performance
function error = evaluate_system(params, G, desired_overshoot, desired_settling_time)
    T1 = params(1);
    T2 = params(2);
    T3 = params(3);
    T4 = params(4);
    K = params(5);
    
    % Lead-Lag Compensator
    D = K * tf([T1, 1], [T2, 1]) * tf([T3, 1], [T4, 1]);
    
    % Compensated System
    Gc = series(D, G);
    Closed_Loop = feedback(Gc, 1);
    
    % Get Step Response Info
    info = stepinfo(Closed_Loop);
    
    % Calculate the error based on overshoot and settling time
    overshoot_error = abs(info.Overshoot - desired_overshoot);
    settling_time_error = abs(info.SettlingTime - desired_settling_time);
    
    % Combine the errors
    error = overshoot_error + settling_time_error;
end
