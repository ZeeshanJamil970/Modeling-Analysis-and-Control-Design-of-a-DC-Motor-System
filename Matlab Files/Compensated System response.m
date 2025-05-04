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

% Display Poles and Zeros of the Open-Loop System
disp('Poles and Zeros of the Open-Loop System:');
disp('Poles:');
disp(pole(G));
disp('Zeros:');
disp(zero(G));

% Root Locus Plot of the Open-Loop System
figure;
rlocus(G);
title('Root Locus of the Open-Loop System');

% Lead-Lag Compensator Design (Initial Guess)
K = 100;      % Gain
T1 = 0.15;    % Lead time constant
T2 = 0.03;    % Lead time constant
T3 = 2.0;     % Lag time constant
T4 = 0.1;     % Lag time constant

% Lead-Lag Compensator Transfer Function
D = K * tf([T1, 1], [T2, 1]) * tf([T3, 1], [T4, 1]);

% Compensated System
Gc = series(D, G);
Closed_Loop = feedback(Gc, 1);

% Display Poles and Zeros of the Compensated System
disp('Poles and Zeros of the Compensated System:');
disp('Poles:');
disp(pole(Closed_Loop));
disp('Zeros:');
disp(zero(Closed_Loop));

% Root Locus Plot of the Compensated System
figure;
rlocus(Gc);
title('Root Locus of the Compensated System');

% Step Response of the Compensated System
figure;
step(Closed_Loop);
title('Step Response of the Compensated System');
grid on;

% Step Info of the Compensated System
info = stepinfo(Closed_Loop);
disp('Compensated System Characteristics:');
disp(info);
