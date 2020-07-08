%%Quanser Qube system initialization
% Control Variants
VSS_CONTROL_BASIC = Simulink.Variant('VSS_CONTROL==0');
VSS_CONTROL_STATE = Simulink.Variant('VSS_CONTROL==1');
VSS_CONTROL_PIDc  = Simulink.Variant('VSS_CONTROL==2');
VSS_CONTROL_PIDp  = Simulink.Variant('VSS_CONTROL==3');

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
thetaMax = 135;  % maximum theta travel (deg) (Qube has +/- 135 deg of travel)
Tf       = 30;   % final time (sec)
Ts       = 0.02; % step time (sec)
maxLimit = 10;   % maximum motor voltage (V)
minLimit = -10;  % minimum motor voltage (V)

% Disturbance 
disturbance = [0 0 0; Tf 0 0];