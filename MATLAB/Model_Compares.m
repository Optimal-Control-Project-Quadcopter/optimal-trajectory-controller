clear;
clc;

%% MODEL PARAMETERS AND CONSTANTS
g  = 9.81;         % ACCELERATION DUE TO GRAVITY
m_unc = 1;         % MASS UNCERTAINTY SCALAR
m  = 2.353598*m_unc;     % MASS OF THE QUADCOPTER IN KG
Ix = 0.1676;       % MOMENT OF INERTIA ALONG X IN KGM^2
Iy = 0.1676;       % MOMENT OF INERTIA ALONG Y IN KGM^2
Iz = 0.29743;      % MOMENT OF INERTIA ALONG Z IN KGM^2
d  = 0.4;          % LENGTH OF THE QUADCOPTER ARM IN M
c  = 1;            % FORCE TO MOMENT SCALING FACTOR 
act_eff = (1-0);     % ACTUATOR EFFICIENCY LOSS SCALAR

%% SYSTEM INITIALISATION
 
% THE STATES OF THE SYSTEM ARE:  x = [φ θ ψ φ' θ' ψ'] 
% WHERE φ -> Roll, θ -> Pitch, AND ψ -> Psi

% THE CONTROL INPUTS TO THE SYSTEM ARE:
% u = [f2 - f4; f1 - f3; f2 + f4 - f1 - f3] WHERE
% f1, f2, f3, f4 ARE THE ROTOR FORCES/THRUSTS

% THE OUTPUTS OF THE SYSTEM ARE:  y = [φ; θ; ψ]

%% SYSTEM INITIALISATION

% The states of the system are:
% x = [x y z x' y' z' φ θ ψ φ' θ' ψ']

NumStates = 12; % Number of states
NumCtrl   = 4; % Number of control inputs
NumOut    = 6; % Number of outputs

A = zeros(NumStates,NumStates);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,8) = -g;
A(5,7) = g;
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(NumStates,NumCtrl);
B(6,1) = 1/m;
B(10,2) = 1/Ix;
B(11,3) = 1/Iy;
B(12,4) = 1/Iz;

RHO = zeros(4,4);
RHO(1,1) = 1;
RHO(2,2) = act_eff;
RHO(3,3) = act_eff;
RHO(4,4) = act_eff;

B_NEW = B*RHO;

C = zeros(NumOut,NumStates);
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;
C(4,7) = 1;
C(5,8) = 1;
C(6,9) = 1;

D = zeros(NumOut,NumCtrl);

% STATE SPACE REPRESENTATION
sysC = ss(A,B_NEW,C,D); 

%% PID Controller Values

att_opt = [125.466100000000	-8.76610000000000	2294];
pitch_opt = [-0.0888000000000000	-0.252600000000000	88.9796000000000];
y_opt = [3.37870000000000e-05	7.51050000000000];
yaw_opt = [1129	692.352300000000];
roll_opt = [-0.0216960721722163	-0.102883770037664	30.1666639334430];
x_opt = [7.75045566817404e-05	3.98251194616216];

%% LQR Controller Values

Q = C'*C;
R = eye(NumCtrl,NumCtrl);
[Klqr,~,~] = lqr(sysC,Q,R);

%% Observer Extension Values

A_CL = A - B_NEW*Klqr;  
B_CL = B_NEW*Klqr;
A_CL_eig = eig(A_CL);

OBSRV_RANK = rank(obsv(A,C)); % CHECK IF THE SYSTEM IS COMPLETELY OBSERVABLE

if OBSRV_RANK == NumStates
    poles = -24*min(abs(real(A_CL_eig))); % DEFINE THE POLES
    add   = 1:length(A_CL_eig); 
    poles = poles - add;
    L     = place(A',C',poles)';        % FIND THE GAINS THROUGH POLE PLACEMENT
    

    % DEFINE THE CLOSED LOOP AUGMENTED SYSTEM
    A_aug_CL = [A_CL, B_CL; zeros(size(A)), A - L*C]; 
    B_aug_CL = [B_NEW; zeros(size(B_NEW))];
    C_aug_CL = [C, zeros(size(C))];
    D_aug_CL = zeros(NumOut,NumCtrl);
    
    sys_aug_CL = ss(A_aug_CL, B_aug_CL, C_aug_CL, D_aug_CL);

end