%% PID & PID CONTROLLERS FOR 6DOF UAV MODEL
% This script presents the ITAE-PID control tuning for a 6-DOF Quadcopter model. 
% 1. The states of the system are the 3 X, Y, and Z (Attitude) positions
% and corresponding velocities, and the 3 Euler angles, and their corresponding 
% anglular rates. 
% 2. The observed outputs are the X, Y, and Z positions and the Roll, Pitch and Yaw angles. 
% 3. There are four inputs U1, U2, U3, and U4 which correspond to total
% upward force, pitch torque, roll torque, and yaw torque
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% To use this code with Simulink simulation open 'Quadcopter_PID_ITAE_6DOF_Optimized_.slx' Simulink file
%% and hit 'Run' from the Editor window.

clear;
clc;

%% Define PID constants for optimization

global time_vect        % Define global time vector for ITAE-PID integration for fminsearch optimization functions
global kp_u1_           % Define global controller U1 Kp constant for U1 fminsearch optimization function & Simulink/MATLAB integration         
global kd_u1_           % Define global controller U1 Kd constant for U1 fminsearch optimization function & Simulink/MATLAB integration
global att_ref_vect     % Define global attitude command reference vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global att_out_vect     % Define global attitude output vector for U1 controller for Simulink/MATLAB integration for U1 fminsearch
global kp_phi_          % Define global controller phi-ref Kp constant for phi-ref fminsearch optimization function & Simulink/MATLAB integration
global kd_phi_          % Define global controller phi-ref Kd constant for phi-ref fminsearch optimization function & Simulink/MATLAB integration
global kp_u2_           % Define global controller U2 Kp constant for U2 fminsearch optimization function & Simulink/MATLAB integration
global ki_u2_           % Define global controller U2 Ki constant for U2 fminsearch optimization function & Simulink/MATLAB integration
global kd_u2_           % Define global controller U2 Kd constant for U2 fminsearch optimization function & Simulink/MATLAB integration
global y_ref_vect       % Define global y command reference vector for phi-ref controller for Simulink/MATLAB integration for phi-ref fminsearch
global y_out_vect       % Define global y output vector for phi-ref controller for Simulink/MATLAB integration for phi-ref fminsearch
global pitch_ref_vect   % Define global pitch command reference vector for U2 controller for Simulink/MATLAB integration for U2 fminsearch
global pitch_out_vect   % Define global pitch output vector for U2 controller for Simulink/MATLAB integration for U2 fminsearch
global kp_theta_        % Define global controller theta-ref Kp constant for theta-ref fminsearch optimization function & Simulink/MATLAB integration
global kd_theta_        % Define global controller theta-ref Kd constant for theta-ref fminsearch optimization function & Simulink/MATLAB integration
global kp_u3_           % Define global controller U3 Kp constant for U3 fminsearch optimization function & Simulink/MATLAB integration 
global ki_u3_           % Define global controller U3 Ki constant for U3 fminsearch optimization function & Simulink/MATLAB integration 
global kd_u3_           % Define global controller U3 Kd constant for U3 fminsearch optimization function & Simulink/MATLAB integration 
global x_ref_vect       % Define global x command reference vector for phi-ref controller for Simulink/MATLAB integration for theta-ref fminsearch
global x_out_vect       % Define global x output vector for phi-ref controller for Simulink/MATLAB integration for theta-ref fminsearch
global roll_ref_vect    % Define global roll command reference vector for U3 controller for Simulink/MATLAB integration for U3 fminsearch
global roll_out_vect    % Define global roll output vector for U3 controller for Simulink/MATLAB integration for U3 fminsearch
global kp_u4_           % Define global controller U4 Kp constant for U4 fminsearch optimization function & Simulink/MATLAB integration   
global kd_u4_           % Define global controller U4 Kd constant for U4 fminsearch optimization function & Simulink/MATLAB integration   
global yaw_ref_vect     % Define global yaw command reference vector for U4 controller for Simulink/MATLAB integration for U4 fminsearch
global yaw_out_vect     % Define global yaw output vector for U4 controller for Simulink/MATLAB integration for U4 fminsearch

%% MODEL PARAMETERS AND CONSTANTS
g  = 9.81;         % acceleration due to gravity
m  = 2.353598;     % mass of the quadcopter in kg
Ix = 0.1676;       % Moment of inertia along x in kgm^2
Iy = 0.1676;       % Moment of inertia along y in kgm^2
Iz = 0.29743;      % Moment of inertia along z in kgm^2
d  = 0.4;          % length of quadcopter arm in m
c  = 1;            % force to moment scaling factor 

%% SYSTEM INITIALISATION

% The states of the system are:
% x = [x y z x' y' z' φ θ ψ φ' θ' ψ']

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

sysC = ss(A,B,C,D); % State space representation

[b1,a1] = ss2tf(A,B,C,D,1);     % Transfer function coefficients for attitude control input excitation
[b2,a2] = ss2tf(A,B,C,D,2);     % Transfer function coefficients for pitch torque control input excitation
[b3,a3] = ss2tf(A,B,C,D,3);     % Transfer function coefficients for roll torque control input excitation
[b4,a4] = ss2tf(A,B,C,D,4);     % Transfer function coefficients for yaw torque control input excitation

u1_z_tf = tf(b1(3,3),[a1(1,1) 0 0]);       % Attitude control input -to- SS output transfer function for z-axis position output for initial tuning/reference
u2_phi_tf = tf(b2(4,3),[a2(1,1) 0 0]);     % Pitch torque control input -to- SS output transfer function for pitch-angle output for initial tuning/reference
u2_y_tf = tf(b2(2,5),[a2(1,1) 0 0]);       % Pitch torque control input -to- SS output transfer function for y-position output for initial tuning/reference     
u3_theta_tf = tf(b3(5,3),[a3(1,1) 0 0]);   % Roll torque control input -to- SS output transfer function for roll-angle output for initial tuning/reference
u3_x_tf = tf(b3(1,5),[a2(1,1) 0 0]);       % Roll torque control input -to- SS output transfer function for x-position output for initial tuning/reference
u4_psi_tf = tf(b4(6,3),[a4(1,1) 0 0]);     % Yaw torque control input -to- SS output transfer function for yaw-angle output for initial tuning/reference

z_ctrl = pidtune(u1_z_tf,'PID',1);          % May use initial values for attitude U1 and yaw U4 controllers from PID tuner function - provide stable guesses
psi_ctrl_ = pidtune(u4_psi_tf,'PID',50);

[Kp_u1, Ki_u1, Kd_u1] = piddata(z_ctrl);        % Initial PD gain constants for U1 controller (Ki unused)
[Kp_u4, Ki_u4, Kd_u4] = piddata(psi_ctrl_);     % Initial PD gain constants for U4 controller (Ki unused)

Kp_u1 = 5.5*Kp_u1;      % Initial PD gain constants after tuning for U1 controller (Ki unused)
Ki_u1 = 1*Ki_u1;
Kd_u1 = 2.5*Kd_u1;

Kp_u4 = 0.52*Kp_u4;     % Initial PD gain constants after tuning for U4 controller (Ki unused)
Ki_u4 = 1*Ki_u4;
Kd_u4 = 0.75*Kd_u4;

Kp_phi = 0.0085;        % Initial PID gain constants after tuning for phi-ref and U2 controllers (Ki unused for phi-ref)
Ki_phi = Kp_phi/5;
Kd_phi = 0.2;
Kp_u2 = 0.1*Kp_phi;
Ki_u2 = 0.0045;
Kd_u2 = 3*Kd_phi;

Kp_theta = 0.0085;      % Initial PID gain constants after tuning for theta-ref and U3 controllers (Ki unused for theta-ref)
Ki_theta = Kp_theta/5;
Kd_theta = 0.2;
Kp_u3 = 0.1*Kp_phi;
Ki_u3 = 0.0045;
Kd_u3 = 3*Kd_phi;

att_init = [Kp_u1 Ki_u1 Kd_u1];         % Initial values vectors to pass to 6 controllers fminsearch functions for ITAE index optimization
y_init = [Kp_phi Kd_phi];
pitch_init = [Kp_u2 Ki_u2 Kd_u2];
x_init = [Kp_theta Kd_theta];
roll_init = [Kp_u3 Ki_u3 Kd_u3];
yaw_init = [Kp_u4 Kd_u4];

kp_u1_ = att_init(1);       % Initialize global PID constant variables to initial guesses for Simulink model
kd_u1_ = att_init(3);
kp_phi_ = y_init(1);
kd_phi_ = y_init(2);
kp_u2_ = pitch_init(1);
ki_u2_ = pitch_init(2);
kd_u2_ = pitch_init(3);
kp_theta_ = x_init(1);
kd_theta_ = x_init(2);
kp_u3_ = roll_init(1);
ki_u3_ = roll_init(2);
kd_u3_ = roll_init(3);
kp_u4_ = yaw_init(1);
kd_u4_= yaw_init(2);

options1 = optimset('TolFun',0.1e-1,'TolX',0.1e-1);         % Adjusted tolerances to improve computation times - after a certain tolerance there is no
options2 = optimset('TolFun',0.001e-1,'TolX',0.001e-1);     % obvious improvement in performance despite fminsearch still iterating solutions
options3 = optimset('TolFun',2e-1,'TolX',2e-1);

%% Optimized values from ITAE performance index minimization

att_opt = [125.466100000000	-8.76610000000000	2294];
pitch_opt = [-0.0888000000000000	-0.252600000000000	88.9796000000000];
y_opt = [3.37870000000000e-05	7.51050000000000];
yaw_opt = [1129	692.352300000000];
roll_opt = [-0.0216960721722163	-0.102883770037664	30.1666639334430];
x_opt = [7.75045566817404e-05	3.98251194616216];

% att_opt = fminsearch(@fn_att_itae,att_init,options1);           % fminsearch function call to fn_att_itae function defined at end of script to tune U1 controller
% y_opt = fminsearch(@fn_y_itae,y_init,options2);                 % fminsearch function call to fn_y_itae function defined at end of script to tune phi-ref controller
% pitch_opt = fminsearch(@fn_pitch_itae,pitch_init,options2);     % fminsearch function call to fn_pitch_itae function defined at end of script to tune U2 controller
% x_opt = fminsearch(@fn_x_itae,x_init,options2);                 % fminsearch function call to fn_x_itae function defined at end of script to tune theta-ref controller
% roll_opt = fminsearch(@fn_roll_itae,roll_init,options2);        % fminsearch function call to fn_roll_itae function defined at end of script to tune U3 controller
% yaw_opt = fminsearch(@fn_yaw_itae,yaw_init,options3);           % fminsearch function call to fn_yaw_itae function defined at end of script to tune U4 controller
 
function att_itae = fn_att_itae(att_init)       % fminsearch optimization function for attitude U1 controller
    global kp_u1_           % Global PID gain constant values
    global ki_u1_
    global kd_u1_
    global att_ref_vect     % Global attitude reference command & output vectors to pull from Simulink simulation
    global att_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp_u1_ = att_init(1);       % Initialize PID gain constants for fminsearch function
    ki_u1_ = att_init(2);
    kd_u1_ = att_init(3);
    t_0 = 0;                    % Simulation time initial value
    t_f = 2;                    % Simulation time final value
    k = t_f/1000;               % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);    % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);        % Run Simulink simulation & pull data vectors
    att_data = out.z_err.Data;      % Attitude reference command/output vectors
    att_itae = 0;                   % Initial values for ITAE performance index before computations
    att_ref_vect = att_data(:,1);       % Attitude reference command vector
    att_out_vect = att_data(:,2);       % Attitude output vector
    l = length(time_vect);              % for-loops for applying Simpson's 1/3rd integration rule
    att_itae = time_vect(l)*abs(att_ref_vect(l)-att_out_vect(l));
    for i = 2:2:l-1
        att_itae = att_itae + 4*time_vect(i)*abs(att_ref_vect(i)-att_out_vect(i));
    end
    for i = 3:2:l-2
        att_itae = att_itae + 2*time_vect(i)*abs(att_ref_vect(i)-att_out_vect(i));
    end
    att_itae = (k/3)*(att_itae);        % Return ITAE performance index value for current fminsearch optimization iteration
end    
    
function y_itae = fn_y_itae(y_init)                % fminsearch optimization function for phi-ref controller
    global kp_phi_          % Global PD gain constant values
    global kd_phi_
    global y_ref_vect       % Global y-position reference command & output vectors to pull from Simulink simulation
    global y_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp_phi_ = y_init(1);        % Initialize PID gain constants for fminsearch function
    kd_phi_ = y_init(2);
    t_0 = 0;                % Simulation time initial value
    t_f = 2;                % Simulation time final value
    k = t_f/1000;           % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);            % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);        % Run Simulink simulation & pull data vectors
    y_data = out.y_err.Data;        % Y-position reference command/output vectors
    y_itae = 0;                     % Initial values for ITAE performance index before computations
    y_ref_vect = y_data(:,1);       % Y-position reference command vector
    y_out_vect = y_data(:,2);       % Y-position output vector 
    l = length(time_vect);
    y_itae = time_vect(l)*abs(y_ref_vect(l)-y_out_vect(l));
    for i = 2:2:l-1                                         % for-loops for applying Simpson's 1/3rd integration rule
        y_itae = y_itae + 4*time_vect(i)*abs(y_ref_vect(i)-y_out_vect(i));
    end
    for i = 3:2:l-2
        y_itae = y_itae + 2*time_vect(i)*abs(y_ref_vect(i)-y_out_vect(i));
    end
    y_itae = (k/3)*(y_itae);            % Return ITAE performance index value for current fminsearch optimization iteration
end   

function pitch_itae = fn_pitch_itae(pitch_init)                % fminsearch optimization function for pitch torque U2 controller
    global kp_u2_               % Global PID gain constant values
    global ki_u2_
    global kd_u2_
    global pitch_ref_vect       % Global pitch-ref reference command & output vectors to pull from Simulink simulation
    global pitch_out_vect
    global time_vect            % Global time vector for Simulink simulation/function integration
    kp_u2_ = pitch_init(1);     % Initialize PID gain constants for fminsearch function
    ki_u2_ = pitch_init(2);
    kd_u2_ = pitch_init(3);
    t_0 = 0;                    % Simulation time initial value
    t_f = 5;                    % Simulation time final value
    k = t_f/1000;               % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);    % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);        % Run Simulink simulation & pull data vectors
    pitch_data = out.phi_err.Data;              % Pitch torque reference command/output vectors
    pitch_itae = 0;                             % Initial values for ITAE performance index before computations   
    pitch_ref_vect = pitch_data(:,1);           % Pitch torque reference command vector
    pitch_out_vect = pitch_data(:,2);           % Pitch torque output vector
    l = length(time_vect);
    pitch_itae = time_vect(l)*abs(pitch_ref_vect(l)-pitch_out_vect(l));
    for i = 2:2:l-1             % for-loops for applying Simpson's 1/3rd integration rule
        pitch_itae = pitch_itae + 4*time_vect(i)*abs(pitch_ref_vect(i)-pitch_out_vect(i));
    end
    for i = 3:2:l-2
        pitch_itae = pitch_itae + 2*time_vect(i)*abs(pitch_ref_vect(i)-pitch_out_vect(i));
    end
    pitch_itae = (k/3)*(pitch_itae);                % Return ITAE performance index value for current fminsearch optimization iteration
end

function x_itae = fn_x_itae(x_init)                    % fminsearch optimization function for theta-ref controller
    global kp_theta_            % Global PD gain constant values
    global kd_theta_            
    global x_ref_vect           % Global x-position reference command & output vectors to pull from Simulink simulation
    global x_out_vect
    global time_vect            % Global time vector for Simulink simulation/function integration
    kp_theta_ = x_init(1);      % Initialize PID gain constants for fminsearch function
    kd_theta_ = x_init(2);
    t_0 = 0;                    % Simulation time initial value
    t_f = 2;                    % Simulation time final value
    k = t_f/1000;               % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);    % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);                % Run Simulink simulation & pull data vectors
    x_data = out.x_err.Data;        % x-position reference command/output vectors
    x_itae = 0;                     % Initial values for ITAE performance index before computations
    x_ref_vect = x_data(:,1);       % x-position reference command vector
    x_out_vect = x_data(:,2);       % x-position output vector
    l = length(time_vect);
    x_itae = time_vect(l)*abs(x_ref_vect(l)+x_out_vect(l));
    for i = 2:2:l-1             % for-loops for applying Simpson's 1/3rd integration rule
        x_itae = x_itae + 4*time_vect(i)*abs(x_ref_vect(i)+x_out_vect(i));
    end
    for i = 3:2:l-2
        x_itae = x_itae + 2*time_vect(i)*abs(x_ref_vect(i)+x_out_vect(i));
    end
    x_itae = (k/3)*(x_itae);                % Return ITAE performance index value for current fminsearch optimization iteration
end   

function roll_itae = fn_roll_itae(roll_init)                   % fminsearch optimization function for pitch torque U3 controller
    global kp_u3_           % Global PID gain constant values
    global ki_u3_
    global kd_u3_
    global roll_ref_vect    % Global roll torque reference command & output vectors to pull from Simulink simulation
    global roll_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp_u3_ = roll_init(1);  % Initialize PID gain constants for fminsearch function
    ki_u3_ = roll_init(2);
    kd_u3_ = roll_init(3);
    t_0 = 0;                % Simulation time initial value
    t_f = 5;                % Simulation time final value
    k = t_f/1000;           % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);            % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);        % Run Simulink simulation & pull data vectors
    roll_data = out.theta_err.Data;     % Roll torque reference command/output vectors
    roll_itae = 0;                      % Initial values for ITAE performance index before computations
    roll_ref_vect = roll_data(:,1);     % Roll torque reference command vector
    roll_out_vect = roll_data(:,2);     % Roll torque output vector
    l = length(time_vect);
    roll_itae = time_vect(l)*abs(roll_ref_vect(l)-roll_out_vect(l));
    for i = 2:2:l-1             % for-loops for applying Simpson's 1/3rd integration rule
        roll_itae = roll_itae + 4*time_vect(i)*abs(roll_ref_vect(i)-roll_out_vect(i));
    end
    for i = 3:2:l-2
        roll_itae = roll_itae + 2*time_vect(i)*abs(roll_ref_vect(i)-roll_out_vect(i));
    end
    roll_itae = (k/3)*(roll_itae);          % Return ITAE performance index value for current fminsearch optimization iteration
end

function yaw_itae = fn_yaw_itae(yaw_init)               % fminsearch optimization function for yaw torque U4 controller
    global kp_u4_           % Global PID gain constant values
    global kd_u4_
    global yaw_ref_vect     % Global yaw torque reference command & output vectors to pull from Simulink simulation
    global yaw_out_vect
    global time_vect        % Global time vector for Simulink simulation/function integration
    kp_u4_ = yaw_init(1);   % Initialize PID gain constants for fminsearch function
    kd_u4_ = yaw_init(2);
    t_0 = 0;                % Simulation time initial value
    t_f = 2;                % Simulation time final value
    k = t_f/1000;           % Simulation time step value for integration steps
    time_vect = (t_0:k:t_f);            % Define time vector for simulation
    out = sim('Quadcopter_PID_ITAE_6DOF',time_vect);        % Run Simulink simulation & pull data vectors
    yaw_data = out.psi_err.Data;            % Yaw torque reference command/output vectors
    yaw_itae = 0;                           % Initial values for ITAE performance index before computations
    yaw_ref_vect = yaw_data(:,1);           % Yaw torque reference command vector
    yaw_out_vect = yaw_data(:,2);           % Yaw torque output vector
    l = length(time_vect);
    yaw_itae = time_vect(l)*abs(yaw_ref_vect(l)-yaw_out_vect(l));
    for i = 2:2:l-1             % for-loops for applying Simpson's 1/3rd integration rule
        yaw_itae = yaw_itae + 4*time_vect(i)*abs(yaw_ref_vect(i)-yaw_out_vect(i));
    end
    for i = 3:2:l-2
        yaw_itae = yaw_itae + 2*time_vect(i)*abs(yaw_ref_vect(i)-yaw_out_vect(i));
    end
    yaw_itae = (k/3)*(yaw_itae);        % Return ITAE performance index value for current fminsearch optimization iteration
end    