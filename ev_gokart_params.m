%% =========================================================================
%% Dr. D. Y. Patil Institute of Technology (DYP DPU) - TEAM PEGASUS
%% EV GO-KART COMPLETE ENGINEERING & POWERTRAIN SPECIFICATION CONFIG
%% Track: Kari Motor Speedway, Coimbatore (2.1 km, 15 Turns)
%% =========================================================================

%% 1. High Voltage (HV) 72V Traction Battery System
params.HV.V_nom = 72.0;               % Nominal pack voltage (V)
params.HV.V_max = 84.0;               % Full charge voltage (84V peak, 20S)
params.HV.V_min = 60.0;               % Cutoff voltage (3.0V/cell x 20S)
params.HV.Cap_Ah_single = 42.0;       % 42 Ah per pack (20S 14P NMC, 3C Rate)
params.HV.Cap_Ah_total = 84.0;        % Total Parallel Capacity = 84 Ah
params.HV.C_rate_cont = 3.0;          % 3C Continuous discharge
params.HV.I_cont_single = 84.0;       % 84 A continuous per pack
params.HV.I_cont_total = 168.0;       % 168 A total continuous
params.HV.I_peak_bms = 200.0;         % 200 A peak BMS limit per pack (400A total)
params.HV.R_int_B1 = 0.024;           % Internal resistance Pack 1 (Ohms)
params.HV.R_int_B2 = 0.024;           % Internal resistance Pack 2 (Ohms)

% Protection Fuses (Bolt-on ANL Fuses)
params.HV.R_F1 = 0.0012;              % 90A 100V DC Bolt-on ANL Balance Fuse (Ohms)
params.HV.R_F2 = 0.0008;              % 150A 100V DC Bolt-on ANL Main Fuse (Ohms)
params.HV.R_cable_40sqmm = 0.0009;    % 40 sq. mm (~1 AWG) Oxygen-Free Copper HV Cable

%% 2. 12V Low Voltage (LV) Auxiliary System
params.LV.V_aux = 12.0;               % 12V Aux battery nominal (V)
params.LV.Cap_Ah = 10.0;              % 10 Ah capacity
params.LV.R_aux_int = 0.015;          % Internal resistance (Ohms)
params.LV.R_switch_on = 0.010;        % Switch contact resistance (Ohms)
params.LV.R_switch_off = 10e6;        % Off resistance (Ohms)

%% 3. Main Isolation Relay / Contactor
params.Contactor.I_rated = 200.0;     % 200 A continuous rating
params.Contactor.V_coil = 12.0;       % 12V DC Coil
params.Contactor.R_coil = 36.0;       % 12V Coil resistance (Ohms)
params.Contactor.L_coil = 0.40;       % 12V Coil inductance (H)
params.Contactor.V_pull_in = 8.0;     % Pull-in threshold voltage (V)
params.Contactor.V_drop_out = 3.5;    % Drop-out voltage (V)
params.Contactor.R_contacts_on = 0.003; % 200A contacts on-resistance (Ohms)
params.Contactor.R_contacts_off = 100e6;% Off-resistance (Ohms)

%% 4. Motor Controller: COMPAGE PMSM BTDS Controller
params.MC.V_max = 80.0;               % Maximum operating voltage (V)
params.MC.I_rated = 133.0;            % Rated continuous phase current (A)
params.MC.I_peak = 350.0;             % Peak phase current limit (350 A)
params.MC.C_dclink = 1200e-6;         % DC-Link capacitance (Farads)
params.MC.ESR_dclink = 0.015;         % DC-Link ESR (Ohms)
params.MC.P_idle = 12.0;              % Controller logic quiescent power (W)
params.MC.V_min_op = 48.0;            % Under-voltage protection limit (V)

%% 5. Traction Motor: COMPAGE PMSM 125/5D (72V, 6 kW)
params.Motor.P_rated = 6000;          % 6 kW rated mechanical power
params.Motor.P_peak = 12000;          % ~12 kW peak power
params.Motor.T_rated = 19.0;          % 19.0 N*m rated torque
params.Motor.T_peak = 61.0;           % 61.0 N*m peak torque
params.Motor.RPM_rated = 3000;        % 3000 RPM rated speed
params.Motor.RPM_max = 6000;          % 6000 RPM maximum speed
params.Motor.Kv = 55.0;               % RPM/V velocity constant
params.Motor.Kt = 0.174;              % Torque constant (N*m/A)
params.Motor.R_phase = 0.018;         % Stator phase resistance (Ohms)
params.Motor.L_phase = 40e-6;         % Stator phase inductance (H)

%% 6. Driver Controls & Safety Loads
params.Throttle.V_min = 0.8;          % 0.8V idle throttle
params.Throttle.V_max = 4.2;          % 4.2V full throttle
params.Loads.R_brake_light = 48.0;    % 12V >=3W LED Brake Light
params.Loads.R_strobe = 48.0;         % 12V >=3W Amber Strobe
params.Loads.R_rtds = 60.0;           % 12V 85-100 dB RTDS Sounder
params.Loads.t_rtds_duration = 2.0;   % 2-second beep
params.Loads.R_display = 40.0;        % 12V Display Unit

%% 7. 420 Chain Drivetrain & Sprocket Calculations
% Driving Pinion: T1 = 13 teeth, D1 = 53.067 mm, Do1 = 59.467 mm
% Driven Sprocket: T2 = 29 teeth, D2 = 117.463 mm, Do2 = 123.863 mm
% 420 Chain: Pitch p = 12.7 mm, Width = 7.9 mm, Roller d1 = 8.0 mm
% Center Distance X = 300 mm, Links k = 68 links, Length L = 863.6 mm
params.Kart.T_pinion = 13;            % 13 teeth driving pinion
params.Kart.T_sprocket = 29;          % 29 teeth driven axle sprocket
params.Kart.gear_ratio = 29.0 / 13.0; % GR = 2.23077 Reduction Ratio
params.Kart.chain_pitch_mm = 12.7;    % 12.7 mm pitch
params.Kart.center_dist_mm = 300.0;   % 300 mm center distance
params.Kart.chain_links = 68;         % 68 links
params.Kart.chain_length_mm = 863.6;  % 863.6 mm exact chain length

%% 8. Vehicle Dynamics, Chassis & Aerodynamic Specs
% Dimensions: L = 1795 mm, W = 1299.26 mm, H = 1.1 m, Ground clearance = 40 mm
params.Kart.mass = 180.0;             % Total mass with driver = 180 kg
params.Kart.wheel_radius_rear = 0.1397; % 11-inch rear racing tyre radius = 0.1397 m (140 mm)
params.Kart.wheel_radius_front = 0.1270;% 10-inch front racing tyre radius = 0.1270 m (127 mm)
params.Kart.wheel_radius = 0.1397;    % Drive wheel radius (m)
params.Kart.Cd = 0.90;                % Aerodynamic drag coefficient
params.Kart.frontal_area = 0.20;      % Frontal area A = 0.20 m^2 (1.3m x 0.15m)
params.Kart.rho_air = 1.20;           % Air density (kg/m^3)
params.Kart.C_rr = 0.010;             % Rolling resistance coefficient
params.Kart.g = 9.81;

%% 9. Axle & Braking Mechanical Engineering Specs (AISI 4130)
params.Axle.D_outer_mm = 30.0;        % Outer diameter D = 30 mm (0.03 m)
params.Axle.d_inner_mm = 15.0;        % Inner diameter d = 15 mm (0.015 m)
params.Axle.length_m = 1.0;           % Axle length = 1.0 m
params.Axle.mass_kg = 4.16;           % Hollow axle mass = 4.16 kg (density 7850 kg/m^3)
params.Axle.yield_strength_MPa = 460; % AISI 4130 Yield Strength = 460 MPa
params.Axle.bending_moment_Nm = 228.0;% Bending moment M = 228 N*m (W*L/4)
params.Axle.torque_Nm = 165.0;        % Torsional braking torque T = 165 N*m
params.Axle.T_eq_Nm = 281.4;          % Teq = sqrt(M^2 + T^2) = 281.4 N*m
params.Axle.tau_eq_MPa = 56.626;      % Equivalent shear stress = 56.626 MPa
params.Axle.FOS = 4.06;               % Factor of Safety = 4.06

params.Brake.mu = 0.8;                % Tyre-road friction coefficient
params.Brake.m_rear = 150.0;          % Rear axle normal load = 150 kg
params.Brake.F_locking = 0.8 * 150 * 9.81; % 1177.2 N rear locking force
params.Brake.T_locking = 1177.2 * 0.1397;  % 164.45 N*m rear braking torque

disp('===========================================================');
disp('  DYP DPU PEGASUS - FULL 180kg / 6kW CALCULATION SPECS LOADED');
disp('===========================================================');
