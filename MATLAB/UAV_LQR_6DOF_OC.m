%% LQR-PI (Observer) FOR 6-DOF QUADCOPTER MODEL
% This script presents the LQR control for a 3-DOF Quadcopter model. 
% The observed outputs are the Roll, Pitch and Yaw angles. There are two
% variations presented:
% 1. The Roll and Pitch angles are varied as a step function and the Yaw
% angle is kept a constant.
% 2. All the angles are varied linearly from a starting value to a final
% value.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% To use this code with Simulink simulation open 'Quadcopter_LQR_PI_Observer_6DOF.slx' Simulink file
%% and hit 'Run' from the Editor window.

clear;
clc;

%% MODEL PARAMETERS AND CONSTANTS
g  = 9.81;         % ACCELERATION DUE TO GRAVITY
m  = 2.353598;     % MASS OF THE QUADCOPTER IN KG
Ix = 0.1676;       % MOMENT OF INERTIA ALONG X IN KGM^2
Iy = 0.1676;       % MOMENT OF INERTIA ALONG Y IN KGM^2
Iz = 0.29743;      % MOMENT OF INERTIA ALONG Z IN KGM^2
d  = 0.4;          % LENGTH OF THE QUADCOPTER ARM IN M
c  = 1;            % FORCE TO MOMENT SCALING FACTOR 

%% SYSTEM INITIALISATION

% The states of the system are:
% x = [x y z x' y' z' φ θ ψ φ' θ' ψ']
% Where x -> x-position, y -> y-position, z -> attitude,
% φ -> Roll, θ -> Pitch, ψ -> Yaw and x' y' z' respective position
% velocities and φ' θ' ψ' Euler angle rates

% The control inputs to the system are:
% u = [U1 U2 U3 U4] where U1 is total upward force, U2 pitch torque, U3
% roll torque, and U4 yaw torque

% The outputs of the system are y = [x; y; z; φ; θ; ψ]

NumStates = 12; % Number of states
NumCtrl   = 4; % Number of control inputs
NumOut    = 6; % Number of outputs

A = zeros(NumStates,NumStates);     % Define A matrix for system state-space model
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,8) = -g;
A(5,7) = g;
A(7,10) = 1;
A(8,11) = 1;
A(9,12) = 1;

B = zeros(NumStates,NumCtrl);       % Define B matrix for system state-space model
B(6,1) = 1/m;
B(10,2) = 1/Ix;
B(11,3) = 1/Iy;
B(12,4) = 1/Iz;

C = zeros(NumOut,NumStates);        % Define C matrix for system state-space model
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;
C(4,7) = 1;
C(5,8) = 1;
C(6,9) = 1;

D = zeros(NumOut,NumCtrl);          % Define empty D matrix for system state-space model

% STATE SPACE REPRESENTATION
sysC = ss(A,B,C,D); 

% FIND POLES OF OPEN LOOP SYSTEM
P_OL = eig(A);

% FIND THE CONTROLLABILITY MATRIX
CTRB_DIM_OL = size(ctrb(A,B));
CTRB_OL     = rank(ctrb(A,B));

% Determine if open-loop quadcopter state-space system is correct &
% controllable

if CTRB_OL == CTRB_DIM_OL(1)
    disp("CONTROLLABLE SYSTEM")
end

%% LQR ALGORITHM 

% DEFINE LQR PARAMETERS AND FIND GAIN
Q = C'*C;
R = 1;
[Klqr,~,~] = lqr(sysC,Q,R);

% FIND THE EIGENVALUES OF THE CLOSED LOOP SYSTEM
A_CL = A - B*Klqr;  
B_CL = B*Klqr;
A_CL_eig = eig(A_CL);

% TO FIND THE GAIN OF THE AUGMENTED SYSTEM(STATES + ERROR), USE POLE 
% PLACEMENT OR ACKERMANN'S FORMULA. THE POLES OF THE OBSERVED SYSTEM ARE 
% PLACED AT LEAST 10x FROM THE ORIGINAL CLOSED LOOP SYSTEM (A-BK)
OBSRV_RANK = rank(obsv(A,C)); % CHECK IF THE SYSTEM IS COMPLETELY OBSERVABLE

if OBSRV_RANK == NumStates
    poles = -24*min(abs(real(A_CL_eig))); % DEFINE THE POLES
    add   = 1:length(A_CL_eig); 
    poles = poles - add;
    L     = place(A',C',poles)';        % FIND THE GAINS THROUGH POLE PLACEMENT
    

    % DEFINE THE CLOSED LOOP AUGMENTED SYSTEM
    A_aug_CL = [A_CL, B_CL; zeros(size(A)), A - L*C]; 
    B_aug_CL = [B; zeros(size(B))];
    C_aug_CL = [C, zeros(size(C))];
    D_aug_CL = zeros(NumOut,NumCtrl);
    
    sys_aug_CL = ss(A_aug_CL, B_aug_CL, C_aug_CL, D_aug_CL);

%     SIMULATION PARAMETERS
%     N = 100;
%     fs = 1;
%     t = 0:1/fs:N-1/fs;
% 
%     %  ------- CASE 1 (CONSTANT FUNCTION) -------
%     Ref_comm = ones(length(t),3);
% 
    %  ------- CASE 2 (STEP FUNCTION) -------
%     Ref_comm(1:length(t),1) = 10;
%     Ref_comm(1:length(t)/2,2) = 0.1; Ref_comm(length(t)/2+1:length(t),2) = 2;
%     Ref_comm(1:length(t)/2,3) = 0.1; Ref_comm(length(t)/2+1:length(t),3) = 1;
%     Ref_comm(1:length(t)/2,4) = 0.1; Ref_comm(length(t)/2+1:length(t),4) = 0.1;

    % ------- CASE 3 (SINUSOID FUNCTION) ------- (DOES NOT WORK!)
    fs       = 512;              % SAMPLING FREQUENCY (SAMPLES PER SECOND)
    dt       = 1/fs;             % SECONDS PER SAMPLE 
    StopTime = 0.25;             % SECONDS
    tTot     = (0:dt:StopTime)'; % SECONDS
    F        = 0.05;               % SINE WAVE FREQUENCY (HERTZ)
    T        = 5/F ;             % TOTAL TIME FOR 5 PERIODS
    t       = 0:dt:T+dt ;       % TIME STEP FOR 5 PERIODS
    Ref_comm(1:length(t),1) = 10;
    Ref_comm(1:length(t),2) = sin(2*pi*F*t);
    Ref_comm(1:length(t),3) = sin(2*pi*F*t + pi/2);
    Ref_comm(1:length(t),4) = sin(2*pi*F*t + pi);

     %  ------- CASE 4 (EXPONENTIAL DECAY FUNCTION) -------
%      Ref_comm(1:length(t),1) = 1 + exp(-t);
%      Ref_comm(1:length(t),2) = 1 + exp(-t./2);
%      Ref_comm(1:length(t),3) = 1 + exp(-t) + exp(-t./2);
%     
%     % INITIAL CONDITION
    x0 = zeros(2*NumStates,1);
% 
    % RUN THE SIMULATION
    [output_CL,time_CL,state_CL] = lsim(sys_aug_CL,Ref_comm,t,x0);
end

%% ERROR ANALYSIS
n = length(time_CL);  % TOTAL NUMBER OF SAMPLES

% COMPUTE THE ERRORS
% E_roll = output_CL(:,4) - Ref_comm(:,2);
% E_pitch = output_CL(:,5) - Ref_comm(:,3);
% E_yaw = output_CL(:,6) - Ref_comm(:,4);

% % COMPUTE THE MEAN SQUARED ERROR (MSE)
% MSE_roll = sum(E_roll.^2)/n;
% MSE_pitch = sum(E_pitch.^2)/n;
% MSE_yaw = sum(E_yaw.^2)/n;

%% PLOTS
figure(1)
plot(time_CL,state_CL(:,3),'LineWidth',2,'LineStyle','--');
hold on
plot(time_CL,Ref_comm(:,1),'LineWidth',2);
title('Attitude Position (z)','FontSize',24)
xlabel('Time (s)','FontSize',24); ylabel('Amplitude (meters)','FontSize',24)
legend('Estimated','Desired','Orientation','horizontal')
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--';
ax.FontSize = 24;
box on

figure(2)
plot(time_CL,state_CL(:,7),'LineWidth',2,'LineStyle','--');
hold on
plot(time_CL,Ref_comm(:,2),'LineWidth',2);
title('Roll angle (\phi)','FontSize',24)
xlabel('Time (s)','FontSize',24); ylabel('Amplitude (radians)','FontSize',24)
legend('Estimated','Desired','Orientation','horizontal')
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--';
ax.FontSize = 24;
box on

figure(3)
plot(time_CL,state_CL(:,8),'LineWidth',2,'LineStyle','--');
hold on
plot(time_CL,Ref_comm(:,3),'LineWidth',2);
title('Pitch angle (\theta)','FontSize',24)
xlabel('Time (s)','FontSize',24); ylabel('Amplitude (radians)','FontSize',24)
legend('Estimated','Desired','Orientation','horizontal')
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--';
ax.FontSize = 24;
box on

figure(4)
plot(time_CL,state_CL(:,9),'LineWidth',2,'LineStyle','--');
hold on
plot(time_CL,Ref_comm(:,4),'LineWidth',2);
title('Yaw angle (\psi)','FontSize',24)
xlabel('Time (s)','FontSize',24); ylabel('Amplitude (radians)','FontSize',24)
legend('Estimated','Desired','Orientation','horizontal')
ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.GridLineStyle = '--';
ax.FontSize = 24;
box on

% figure(4)
% plot(abs(E_roll),'LineWidth',2); hold on; plot(abs(E_pitch),'LineWidth',2); 
% hold on; plot(abs(E_yaw),'LineWidth',2)
% title('Absolute Error','FontSize',24)
% xlabel('Time (s)','FontSize',24); ylabel('Magnitude (radians)','FontSize',24)
% legend('Roll','Pitch','Yaw','Orientation','horizontal')
% ax = gca;
% ax.XGrid = 'on';
% ax.YGrid = 'on';
% ax.GridLineStyle = '--';
% ax.FontSize = 24;
% box on