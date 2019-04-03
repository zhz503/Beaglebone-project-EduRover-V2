% EduRover Parameters and State-Space Matrix Calculation

% Physical Constant
g = 9.81;						% gravity acceleration[m/sec^2]

% Rover Parameters
m = 0.03852; 						% wheel weight [kg]
R = 0.085/2;  						% wheel radius [m]
Jw = m * R^2 / 2;				% wheel inertia moment [kgm^2]
M = 0.6; 						% body weight [kg]
W = 0.15;					% body width [m]
D = 0.06;						% body depth [m]
H = 0.09;						% body height [m]
L = 0.8869; 				% distance of the center of mass from the wheel axle [m]
Jpsi = M * L^2 / 3;				% body pitch inertia moment [kgm^2]
Jphi = M * (W^2 + D^2) / 12;	% body yaw inertia moment [kgm^2]

fm = 0.0022;					% friction coefficient between body & DC motor
fw = 0.1;							% friction coefficient between wheel & floor

% DC Motor Parameters
 Tor= 0.1765387958333325;  % Motor torque [Nm]
Im = 1.6;                      % Motor working current [A]
Jm = 2.7565e-7 ;  				% DC motor inertia moment [kgm^2]
Rm = 2.7;						% DC motor resistance [ohm]
Kb = 7.4074e-04;                     % DC motor back EMF constant [Vsec/rad]
Kt = Tor/ Im;                     % DC motor torque constant [Nm/A]
n = 99;							% gear ratio

% EduRover State-Space Matrix Calculation
alpha = n * Kt / Rm;
beta = n * Kt * n* Kb / Rm + fm;
tmp = beta + fw;

E_11 = (2 * m + M) * R^2 + 2 * Jw + 2 * n^2 * Jm;
E_12 = M * L * R - 2 * n^2 * Jm;
E_22 = M * L^2 + Jpsi + 2 * n^2 * Jm;
detE = E_11 * E_22 - E_12^2;

A1_32 = -g * M * L * E_12 / detE;
A1_42 = g * M * L * E_11 / detE;
A1_33 = -1 * (tmp * E_22 + beta * E_12) / detE;
A1_43 =  (tmp * E_12 + beta * E_11) / detE;
A1_34 =  beta * (E_22 + E_12) / detE;
A1_44 = - beta * (E_11 + E_12) / detE;
B1_3 = alpha * (E_22 + E_12) / detE;
B1_4 = -alpha * (E_11 + E_12) / detE;
A1 = [
	0 0 1 0
	0 0 0 1
	0 A1_32 A1_33 A1_34
	0 A1_42 A1_43 A1_44
	];
B1 = [
	0 0
	0 0
	B1_3 B1_3
	B1_4 B1_4
	];
C1 = eye(4);
D1 = zeros(4, 2);

I = m * W^2 / 2 + Jphi + (Jw + n^2 * Jm) * W^2 / (2 * R^2);
J = tmp ;
K = alpha * W /  R;
A2 = [
	0 1
	0 -J / I
	];
B2 = [
	0      0
	-K / I K / I
	];
C2 = eye(2);
D2 = zeros(2);
clear alpha beta tmp
clear E_11 E_12 E_22 detE
clear A1_32 A1_33 A1_34 A1_42 A1_43 A1_44 B1_3 B1_4 I J K
% Controller Parameters 

% Servo Gain Calculation using Optimal Regulator
A_BAR = [A1, zeros(4, 1); C1(1, :), 0];
B_BAR = [B1; 0, 0];
QQ = [
	1, 0,   0, 0, 0
	0, 6e5, 0, 0, 0
	0, 0,   1, 0, 0
	0, 0,   0, 1, 0
	0, 0,   0, 0, 4e2
	];
RR = 1e3 * eye(2);
KK = lqr(A_BAR, B_BAR, QQ, RR);
k_f = KK(1, 1:4);					% feedback gain
k_i = KK(1, 5);						% integral gain

ts1 = 0.002;
a_r = 0.996;						% smooth reference signal
dt = 0.005;

