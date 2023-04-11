%% PID CONTROLLERS FOR 3-DOF QUADCOPTER MODEL
% This script presents the ITAE-PID control for a 3-DOF Quadcopter model.
% 1. The 6 states of the system are the 3 Euler angles φ θ ψ , and their corresponding 
% anglular rates φ' θ' ψ'.
% 2. The observed outputs are the Roll φ, Pitch θ and Yaw  ψ angles.
% 3. There are 4 inputs due to the 4 rotors of the Quadcopter. However, we 
% consider 3 controller inputs, U1, U2, and U3 which are functions of the 4
% rotor thrust forces F1-F4; U1 = F1-F4, U2 = F1-F3 & U3 = F2+F4-F1-F3 based on how 
% they affect the states of the system  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% To use this code with Simulink simulation open 'Quadcopter_PID_ITAE_3DOF.slx' Simlunk file
%% and hit 'Run' from the Editor window.

clear;
clc;

%% Define PID constants for optimization

global time_vect            % Define global time vector for ITAE-PID integration for fminsearch optimization functions
global kp1                  % Define global controller U1 Kp constant for U1 fminsearch optimization function & Simulink/MATLAB integration  
global ki1                  % Define global controller U1 Ki constant for U1 fminsearch optimization function & Simulink/MATLAB integration  
global kd1                  % Define global controller U1 Kd constant for U1 fminsearch optimization function & Simulink/MATLAB integration  
global roll_ref_vect        % Define global roll command reference vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global roll_out_vect        % Define global roll output vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global kp2                  % Define global controller U2 Kp constant for U2 fminsearch optimization function & Simulink/MATLAB integration  
global ki2                  % Define global controller U2 Ki constant for U2 fminsearch optimization function & Simulink/MATLAB integration  
global kd2                  % Define global controller U2 Kd constant for U2 fminsearch optimization function & Simulink/MATLAB integration  
global pitch_ref_vect       % Define global roll pitch reference vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global pitch_out_vect       % Define global roll output vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global kp3                  % Define global controller U3 Kp constant for U3 fminsearch optimization function & Simulink/MATLAB integration  
global ki3                  % Define global controller U3 Ki constant for U3 fminsearch optimization function & Simulink/MATLAB integration  
global kd3                  % Define global controller U3 Kd constant for U1 fminsearch optimization function & Simulink/MATLAB integration  
global yaw_ref_vect         % Define global yaw command reference vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global yaw_out_vect         % Define global yaw output vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch

%% MODEL PARAMETERS AND CONSTANTS
g  = 9.81;         % ACCELERATION DUE TO GRAVITY
m  = 2.353598;     % MASS OF THE QUADCOPTER IN KG
Ix = 0.1676;       % MOMENT OF INERTIA ALONG X IN KGM^2
Iy = 0.1676;       % MOMENT OF INERTIA ALONG Y IN KGM^2
Iz = 0.29743;      % MOMENT OF INERTIA ALONG Z IN KGM^2
d  = 0.4;          % LENGTH OF THE QUADCOPTER ARM IN M
c  = 1;            % FORCE TO MOMENT SCALING FACTOR

%% SYSTEM INITIALISATION

% THE STATES OF THE SYSTEM ARE:  x = [φ θ ψ φ' θ' ψ']
% WHERE φ -> Roll, θ -> Pitch, AND ψ -> Psi

% THE CONTROL INPUTS TO THE SYSTEM ARE:
% u = [f2 - f4; f1 - f3; f2 + f4 - f1 - f3] WHERE
% f1, f2, f3, f4 ARE THE ROTOR FORCES/THRUSTS

% THE OUTPUTS OF THE SYSTEM ARE:  y = [φ; θ; ψ]

NumStates = 6; % NUMBER OF STATES
NumCtrl   = 3; % NUMBER OF CONTROL INPUTS
NumOut    = 3; % NUMBER OF OUTPUTS

A = zeros(NumStates,NumStates);     % Define A matrix for system state-space model
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B = zeros(NumStates,NumCtrl);       % Define B matrix for system state-space model
B(4,1) = d/Ix;
B(5,2) = d/Iy;
B(6,3) = c/Iz;

C = zeros(NumOut,NumStates);        % Define C matrix for system state-space model
C(1,1) = 1;
C(2,2) = 1;
C(3,3) = 1;

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

%% PID ALGORITHM

% CREATE SUBSYSTEMS
subsys1 = tf(ss(A,B,C(1,:),D(1,:)));  % Transfer function coefficients for roll angle control input excitation
subsys1 = tf(subsys1.Numerator(1),subsys1.Denominator(1));
subsys2 = tf(ss(A,B,C(2,:),D(2,:)));  % Transfer function coefficients for pitch angle control input excitation
subsys2 = tf(subsys2.Numerator(2),subsys2.Denominator(2));
subsys3 = tf(ss(A,B,C(3,:),D(3,:)));  % Transfer function coefficients for yaw angle control input excitation
subsys3 = tf(subsys3.Numerator(3),subsys3.Denominator(3));

% CREATE 3 PID CONTROLLERS, ONE FOR EACH SUBSYSTEM - Values used for
% Initialization of ITAE PID tuning gains
K_p1 = 2; K_d1 = 3.6; K_i1 = 0.4;       % Initial PID gain constants after tuning for U1 controller
K_p2 = 2; K_d2 = 3.6; K_i2 = 0.4;       % Initial PID gain constants after tuning for U2 controller
K_p3 = 2; K_d3 = 2; K_i3 = 1;           % Initial PID gain constants after tuning for U3 controller

pitch_init = [K_p1 K_i1 K_d1];          % Initial values vectors to pass to 3 controllers fminsearch functions for ITAE index optimization         
roll_init = [K_p2 K_i2 K_d2];
yaw_init = [K_p3 K_i3 K_d3];

kp1 = pitch_init(1);            % Initialize global PID constant variables to initial guesses for Simulink model
ki1 = pitch_init(2);
kd1 = pitch_init(3);
kp2 = roll_init(1);
ki2 = roll_init(2);
kd2 = roll_init(3);
kp3 = yaw_init(1);
ki3 = yaw_init(2);
kd3 = yaw_init(3);

t_f = 10;        % Common simulation final time for all controller optimization functions
k = 20e-3;      % Simulation/integration time step

%% Optimized values from ITAE performance index minimization

roll_opt = fminsearch(@fn_roll_itae,roll_init,[],t_f,k);            % fminsearch function call to fn_roll_itae function defined at end of script to tune U1 controller
pitch_opt = fminsearch(@fn_pitch_itae,pitch_init,[],t_f,k);         % fminsearch function call to fn_pitch_itae function defined at end of script to tune U2 controller
yaw_opt = fminsearch(@fn_yaw_itae,yaw_init,[],t_f,k);               % fminsearch function call to fn_yaw_itae function defined at end of script to tune U3 controller

function roll_itae = fn_roll_itae(phi_init,t_f,k)       % fminsearch optimization function for roll angle U1 controller
    global kp1              % Global PID gain constant values
    global ki1
    global kd1
    global roll_ref_vect   % Global roll angle reference command & output vectors to pull from Simulink simulation
    global roll_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp1 = phi_init(1);      % Initialize PID gain constants for fminsearch function
    ki1 = phi_init(2);
    kd1 = phi_init(3);
    t_0 = 0;                % Simulation time initial value
    time_vect = (t_0:k:t_f);    % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_3DOF',time_vect);        % Run Simulink simulation & pull data vectors
    roll_data = out.phi_err.Data;       % Roll angle reference command/output vectors
    roll_itae = 0;                      % Initial values for ITAE performance index before computations
    roll_ref_vect = roll_data(:,1);     % Roll angle reference command vector
    roll_out_vect = roll_data(:,2);     % Roll angle output command vector
    l = length(time_vect);
    roll_itae = time_vect(l)*abs(roll_ref_vect(l)-roll_out_vect(l));
    for i = 2:2:l-1                     % for-loops for applying Simpson's 1/3rd integration rule
        roll_itae = roll_itae + 4*time_vect(i)*abs(roll_ref_vect(i)-roll_out_vect(i));
    end
    for i = 3:2:l-2
        roll_itae = roll_itae + 2*time_vect(i)*abs(roll_ref_vect(i)-roll_out_vect(i));
    end
    roll_itae = (k/3)*(roll_itae);      % Return ITAE performance index value for current fminsearch optimization iteration
end    
    
function pitch_itae = fn_pitch_itae(theta_init,t_f,k)       % fminsearch optimization function for pitch angle U2 controller
    global kp2                  % Global PID gain constant values
    global ki2
    global kd2
    global pitch_ref_vect       % Global pitch reference command & output vectors to pull from Simulink simulation
    global pitch_out_vect       
    global time_vect            % Global time vector for Simulink simulation/function integration
    kp2 = theta_init(1);        % Initialize PID gain constants for fminsearch function
    ki2 = theta_init(2);
    kd2 = theta_init(3);
    t_0 = 0;                    % Simulation time initial value
    time_vect = (t_0:k:t_f);    % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_3DOF',time_vect);          % Run Simulink simulation & pull data vectors
    pitch_data = out.theta_err.Data;            % Pitch angle reference command/output vectors
    pitch_itae = 0;                             % Initial values for ITAE performance index before computations
    pitch_ref_vect = pitch_data(:,1);           % Pitch angle reference command vector
    pitch_out_vect = pitch_data(:,2);           % Pitch angle output command vector
    l = length(time_vect);
    pitch_itae = time_vect(l)*abs(pitch_ref_vect(l)-pitch_out_vect(l));
    for i = 2:2:l-1                             % for-loops for applying Simpson's 1/3rd integration rule
        pitch_itae = pitch_itae + 4*time_vect(i)*abs(pitch_ref_vect(i)-pitch_out_vect(i));
    end
    for i = 3:2:l-2
        pitch_itae = pitch_itae + 2*time_vect(i)*abs(pitch_ref_vect(i)-pitch_out_vect(i));
    end
    pitch_itae = (k/3)*(pitch_itae);            % Return ITAE performance index value for current fminsearch optimization iteration
end

function yaw_itae = fn_yaw_itae(psi_init,t_f,k)         % fminsearch optimization function for yaw angle U3 controller
    global kp3              % Global PID gain constant values
    global ki3
    global kd3
    global yaw_ref_vect     % Global yaw angle reference command & output vectors to pull from Simulink simulation
    global yaw_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp3 = psi_init(1);      % Initialize PID gain constants for fminsearch function
    ki3 = psi_init(2);
    kd3 = psi_init(3);
    t_0 = 0;                % Simulation time initial value
    time_vect = (t_0:k:t_f);        % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_3DOF',time_vect);        % Run Simulink simulation & pull data vectors
    yaw_data = out.psi_err.Data;        % Yaw angle reference command/output vectors
    yaw_itae = 0;                       % Initial values for ITAE performance index before computations
    yaw_ref_vect = yaw_data(:,1);       % Yaw angle reference command vector
    yaw_out_vect = yaw_data(:,2);       % Yaw angle output command vector
    l = length(time_vect);
    yaw_itae = time_vect(l)*abs(yaw_ref_vect(l)-yaw_out_vect(l));
    for i = 2:2:l-1                     % for-loops for applying Simpson's 1/3rd integration rule
        yaw_itae = yaw_itae + 4*time_vect(i)*abs(yaw_ref_vect(i)-yaw_out_vect(i));
    end
    for i = 3:2:l-2
        yaw_itae = yaw_itae + 2*time_vect(i)*abs(yaw_ref_vect(i)-yaw_out_vect(i));
    end
    yaw_itae = (k/3)*(yaw_itae);        % Return ITAE performance index value for current fminsearch optimization iteration
end