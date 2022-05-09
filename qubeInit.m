%%Quanser Qube system initialization
% Control Variants
VSS_CONTROL_BASIC = Simulink.Variant('VSS_CONTROL==0');
VSS_CONTROL_STATE = Simulink.Variant('VSS_CONTROL==1');
VSS_CONTROL_FF    = Simulink.Variant('VSS_CONTROL==2');

VSS_CONTROL = 0; %Initialize to basic controller

% Electrical Parameters
Rm = 8.4;   % armature resistance (Ohm) (given by datasheet)
Lm = 1.16;  % armature inductance (mH) (given by datasheet)
Km = 0.042; % motor back-emf constant (V/(rad/s)) (given by datasheet)
Jm = 4e-6;  % rotor inertia (kg m^2) (given by datasheet)

% Mechanical Parameters
g = 9.81;    % gravitational constant (m/s^2)
Mh = 0.0106; % hub mass (kg) (given by datasheet)
Rh = 0.0111; % hub radius (m) (given by datasheet)
Mr = 0.095;  % arm mass (kg) (given by datasheet)
Lr = 0.085;  % arm length (m) (given by datasheet)
Dr = 5e-4;   % rotary arm damping coefficient (Nm/(rad/s)) (determined by experiment)
Mp = 0.024;  % pendulum mass (kg) (given by datasheet)
Lp = 0.129;  % pendulum length (m) (given by datasheet)
Dp = 2e-5;   % pendulum damping coefficient (Nm/(rad/s)) (determined by experiment)

% Model Conditions
alpha0   = 180;  % initial pendulum angle (deg)
theta0   = 0;    % initial rotary arm angle (deg)
thetaMax = 135;  % maximum theta travel (deg) (Qube has +/- 135 deg of travel)
Tf       = 30;   % final time (sec)
Ts       = 0.02; % step time (sec)
maxLimit = 10;   % maximum motor voltage (V)
minLimit = -10;  % minimum motor voltage (V)

% Disturbance 
disturbance = [0 0 0; Tf 0 0];

% Square Wave demand
stepSize = 0; % size of the size in demand (set to zero initially)
stepFreq = 1; % Frequency of the step

% LQR Controller Definition
A=[0 1 0 0;
0 -0.0104 149.2751 0;
0 0 0 1;
0 -0.0103 261.6091 0];

B = [0;49.7275;0;49.1493];

Q=diag([1,0,5,0]);
R=1;

Klqr = lqr(A,B,Q,R);