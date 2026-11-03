%% =========================================================================
%% Dr. D. Y. Patil Institute of Technology (DYP DPU) - TEAM PEGASUS
%% EV Go-Kart simple Simulink block model generator (180kg / 6kW)
%% Track: Kari Motor Speedway, Coimbatore
%% =========================================================================

clear; clc;

model_name = 'ev_powertrain_gokart';

% Force close if already open
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Create and open new system
new_system(model_name);
open_system(model_name);
set_param(model_name, 'Solver', 'ode23tb', 'StopTime', '100.0');

disp('1. Creating all schematic blocks with DYP DPU PEGASUS specifications...');

%% 1. HIGH VOLTAGE 72V 84Ah BATTERY PACK (PARALLEL 2x 42Ah 3C NMC) & FUSES
add_block('simulink/Sources/Constant', [model_name, '/B1_72V_42Ah'], ...
    'Value', '72', 'Position', [50, 40, 130, 100]);

add_block('simulink/Sources/Constant', [model_name, '/B2_72V_42Ah'], ...
    'Value', '72', 'Position', [170, 40, 250, 100]);

add_block('simulink/Math Operations/MinMax', [model_name, '/Parallel_Bus'], ...
    'Function', 'max', 'Inputs', '2', 'Position', [290, 55, 320, 85]);

add_block('simulink/Math Operations/Gain', [model_name, '/F1_90A_ANL_Fuse'], ...
    'Gain', '1.0', 'Position', [130, 130, 170, 160]);

add_block('simulink/Math Operations/Gain', [model_name, '/F2_150A_ANL_Fuse'], ...
    'Gain', '1.0', 'Position', [400, 55, 440, 85]);

%% 2. RELAY / CONTACTOR (200A 100V DC CONTACTS & 12V COIL)
add_block('simulink/Signal Routing/Switch', [model_name, '/Contactor_200A_Contacts'], ...
    'Threshold', '0.5', 'Position', [500, 110, 540, 170]);

add_block('simulink/Logic and Bit Operations/Compare To Constant', [model_name, '/Contactor_12V_Coil'], ...
    'const', '8.0', 'relop', '>=', 'Position', [490, 220, 550, 260]);

add_block('simulink/Sources/Constant', [model_name, '/HV_Open_0V'], ...
    'Value', '0', 'Position', [420, 150, 460, 170]);

%% 3. MOTOR CONTROLLER: COMPAGE PMSM BTDS (80V, 133A RATED, 350A PEAK)
sub_mc = [model_name, '/COMPAGE_BTDS_Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_mc, ...
    'Position', [620, 40, 780, 200]);
delete_line(sub_mc, 'In1/1', 'Out1/1');
delete_block([sub_mc, '/In1']);
delete_block([sub_mc, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_mc, '/In_HV_POS'], 'Position', [30, 30, 60, 50]);
add_block('simulink/Ports & Subsystems/In1', [sub_mc, '/In_HV_NEG'], 'Position', [30, 80, 60, 100]);
add_block('simulink/Ports & Subsystems/In1', [sub_mc, '/In_Throttle'], 'Position', [30, 130, 60, 150]);
add_block('simulink/Ports & Subsystems/In1', [sub_mc, '/In_Brake'], 'Position', [30, 180, 60, 200]);

add_block('simulink/Math Operations/Gain', [sub_mc, '/Brake_Invert'], 'Gain', '-1', 'Position', [90, 180, 115, 200]);
add_block('simulink/Math Operations/Bias', [sub_mc, '/Brake_High'], 'Bias', '1', 'Position', [135, 180, 160, 200]);
add_block('simulink/Math Operations/Product', [sub_mc, '/Throttle_Gate'], 'Inputs', '2', 'Position', [190, 130, 220, 165]);
add_block('simulink/Logic and Bit Operations/Compare To Constant', [sub_mc, '/HV_OK'], 'const', '48.0', 'relop', '>=', 'Position', [100, 30, 160, 60]);
add_block('simulink/Math Operations/Product', [sub_mc, '/HV_Gate'], 'Inputs', '2', 'Position', [250, 80, 280, 120]);

add_block('simulink/Ports & Subsystems/Out1', [sub_mc, '/Phase_U'], 'Position', [340, 50, 370, 70]);
add_block('simulink/Ports & Subsystems/Out1', [sub_mc, '/Phase_V'], 'Position', [340, 100, 370, 120]);
add_block('simulink/Ports & Subsystems/Out1', [sub_mc, '/Phase_W'], 'Position', [340, 150, 370, 170]);

add_line(sub_mc, 'In_Brake/1', 'Brake_Invert/1');
add_line(sub_mc, 'Brake_Invert/1', 'Brake_High/1');
add_line(sub_mc, 'In_Throttle/1', 'Throttle_Gate/1');
add_line(sub_mc, 'Brake_High/1', 'Throttle_Gate/2');
add_line(sub_mc, 'In_HV_POS/1', 'HV_OK/1');
add_line(sub_mc, 'HV_OK/1', 'HV_Gate/1');
add_line(sub_mc, 'Throttle_Gate/1', 'HV_Gate/2');
add_line(sub_mc, 'HV_Gate/1', 'Phase_U/1');
add_line(sub_mc, 'HV_Gate/1', 'Phase_V/1');
add_line(sub_mc, 'HV_Gate/1', 'Phase_W/1');

%% 4. MOTOR (M): COMPAGE PMSM 125/5D (72V, 6 kW RATED, 61 Nm PEAK) & 13T:29T CHAIN
sub_motor = [model_name, '/COMPAGE_PMSM_6kW'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_motor, ...
    'Position', [850, 65, 960, 175]);
delete_line(sub_motor, 'In1/1', 'Out1/1');
delete_block([sub_motor, '/In1']);
delete_block([sub_motor, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_motor, '/U'], 'Position', [30, 40, 60, 60]);
add_block('simulink/Ports & Subsystems/In1', [sub_motor, '/V'], 'Position', [30, 80, 60, 100]);
add_block('simulink/Ports & Subsystems/In1', [sub_motor, '/W'], 'Position', [30, 120, 60, 140]);
add_block('simulink/Math Operations/Gain', [sub_motor, '/Peak_Torque_61Nm'], 'Gain', '61.0', 'Position', [100, 75, 150, 105]);
add_block('simulink/Continuous/Transfer Fcn', [sub_motor, '/Inertia_180kg'], 'Numerator', '[1]', 'Denominator', '[0.06 1]', 'Position', [180, 70, 240, 110]);
% 420 Chain Drivetrain: 13T pinion / 29T sprocket -> G = 2.23077, Rear Tire R = 0.1397m (11-inch tyre)
add_block('simulink/Math Operations/Gain', [sub_motor, '/Speed_kmh'], 'Gain', '3.6 * (0.1397 / (29/13))', 'Position', [270, 75, 330, 105]);
add_block('simulink/Ports & Subsystems/Out1', [sub_motor, '/Speed_Out'], 'Position', [360, 80, 390, 100]);

add_line(sub_motor, 'V/1', 'Peak_Torque_61Nm/1');
add_line(sub_motor, 'Peak_Torque_61Nm/1', 'Inertia_180kg/1');
add_line(sub_motor, 'Inertia_180kg/1', 'Speed_kmh/1');
add_line(sub_motor, 'Speed_kmh/1', 'Speed_Out/1');

%% 5. 12V 10Ah AUXILIARY SUPPLY & SAFETY INTERLOCK LOOP
add_block('simulink/Sources/Constant', [model_name, '/Aux_12V_10Ah_Battery'], ...
    'Value', '12', 'Position', [40, 360, 120, 420]);

add_block('simulink/Sources/Step', [model_name, '/Kill_SW1_Mushroom'], ...
    'Time', '0.1', 'Before', '0', 'After', '1', 'Position', [160, 375, 200, 405]);

add_block('simulink/Sources/Step', [model_name, '/Key_SW_22mm'], ...
    'Time', '0.2', 'Before', '0', 'After', '1', 'Position', [240, 375, 280, 405]);

add_block('simulink/Sources/Step', [model_name, '/Kill_SW2_PushPull'], ...
    'Time', '4.5', 'Before', '1', 'After', '0', 'Position', [320, 375, 360, 405]);

add_block('simulink/Sources/Step', [model_name, '/Toggle_SW_10A'], ...
    'Time', '0.5', 'Before', '0', 'After', '1', 'Position', [400, 375, 440, 405]);

add_block('simulink/Math Operations/Product', [model_name, '/Safety_Loop_AND'], ...
    'Inputs', '5', 'Position', [480, 360, 510, 430]);

add_block('simulink/Math Operations/Product', [model_name, '/Kill_SW1_Tap'], ...
    'Inputs', '2', 'Position', [170, 450, 200, 480]);

%% 6. AUXILIARY LOADS & DISPLAY
add_block('simulink/Sources/Step', [model_name, '/Brake_Limit_SW'], ...
    'Time', '1.2', 'Before', '0', 'After', '1', 'Position', [150, 530, 190, 560]);

add_block('simulink/Math Operations/Product', [model_name, '/Brake_Light_Gate'], ...
    'Inputs', '2', 'Position', [220, 525, 250, 555]);

add_block('simulink/Math Operations/Gain', [model_name, '/Brake_Light_LED_3W'], ...
    'Gain', '12', 'Position', [290, 525, 370, 555]);

add_block('simulink/Math Operations/Gain', [model_name, '/Strobe_LED_3W'], ...
    'Gain', '10.0', 'Position', [290, 590, 370, 620]);

sub_disp = [model_name, '/Dashboard_Display'];
add_block('simulink/Ports & Subsystems/Subsystem', sub_disp, ...
    'Position', [290, 660, 390, 780]);
delete_line(sub_disp, 'In1/1', 'Out1/1');
delete_block([sub_disp, '/In1']);
delete_block([sub_disp, '/Out1']);

add_block('simulink/Ports & Subsystems/In1', [sub_disp, '/12V_Pwr'], 'Position', [30, 20, 60, 40]);
add_block('simulink/Ports & Subsystems/In1', [sub_disp, '/In_1_Speed'], 'Position', [30, 60, 60, 80]);
add_block('simulink/Ports & Subsystems/In1', [sub_disp, '/In_2_BattV'], 'Position', [30, 100, 60, 120]);
add_block('simulink/Ports & Subsystems/In1', [sub_disp, '/In_3_Temp'], 'Position', [30, 140, 60, 160]);
add_block('simulink/Ports & Subsystems/In1', [sub_disp, '/In_4_Fault'], 'Position', [30, 180, 60, 200]);
add_block('simulink/Sinks/Display', [sub_disp, '/Telemetry_Display'], 'Position', [120, 90, 200, 130]);
add_line(sub_disp, 'In_1_Speed/1', 'Telemetry_Display/1');

add_block('simulink/Sources/Constant', [model_name, '/In_1_Speed'], 'Value', '45.0', 'Position', [200, 800, 230, 820]);
add_block('simulink/Sources/Constant', [model_name, '/In_2_BattV'], 'Value', '72.0', 'Position', [250, 800, 280, 820]);
add_block('simulink/Sources/Constant', [model_name, '/In_3_Temp'], 'Value', '56.0', 'Position', [300, 800, 330, 820]);
add_block('simulink/Sources/Constant', [model_name, '/In_4_Fault'], 'Value', '0.0', 'Position', [350, 800, 380, 820]);

%% 7. FOOT CONTROLS, TIMER MODULE & RTDS (85-100 dB)
add_block('simulink/Sources/Ramp', [model_name, '/Hall_Throttle_0.8_4.2V'], ...
    'slope', '0.4', 'start', '2.0', 'Position', [520, 470, 590, 500]);

add_block('simulink/Sources/Step', [model_name, '/Brake_Pedal'], ...
    'Time', '3.2', 'Before', '0', 'After', '1', 'Position', [640, 470, 710, 500]);

add_block('simulink/Discrete/Discrete-Time Integrator', [model_name, '/Timer_Module_2s'], ...
    'Position', [480, 680, 540, 720]);

add_block('simulink/Math Operations/Gain', [model_name, '/RTDS_Sounder_90dB'], ...
    'Gain', '12.0', 'Position', [600, 685, 720, 715]);

%% =========================================================================
%% 8. CONNECT ALL TOP-LEVEL WIRES VIA PORTHANDLES
%% =========================================================================
disp('2. Connecting all wire branches via PortHandles...');

ph_b1      = get_param([model_name, '/B1_72V_42Ah'], 'PortHandles');
ph_b2      = get_param([model_name, '/B2_72V_42Ah'], 'PortHandles');
ph_bus     = get_param([model_name, '/Parallel_Bus'], 'PortHandles');
ph_f1      = get_param([model_name, '/F1_90A_ANL_Fuse'], 'PortHandles');
ph_f2      = get_param([model_name, '/F2_150A_ANL_Fuse'], 'PortHandles');
ph_cont    = get_param([model_name, '/Contactor_200A_Contacts'], 'PortHandles');
ph_coil    = get_param([model_name, '/Contactor_12V_Coil'], 'PortHandles');
ph_hv0     = get_param([model_name, '/HV_Open_0V'], 'PortHandles');
ph_mc      = get_param([model_name, '/COMPAGE_BTDS_Controller'], 'PortHandles');
ph_motor   = get_param([model_name, '/COMPAGE_PMSM_6kW'], 'PortHandles');

ph_aux     = get_param([model_name, '/Aux_12V_10Ah_Battery'], 'PortHandles');
ph_k1      = get_param([model_name, '/Kill_SW1_Mushroom'], 'PortHandles');
ph_key     = get_param([model_name, '/Key_SW_22mm'], 'PortHandles');
ph_k2      = get_param([model_name, '/Kill_SW2_PushPull'], 'PortHandles');
ph_tog     = get_param([model_name, '/Toggle_SW_10A'], 'PortHandles');
ph_safety  = get_param([model_name, '/Safety_Loop_AND'], 'PortHandles');
ph_k1tap   = get_param([model_name, '/Kill_SW1_Tap'], 'PortHandles');

ph_bsw     = get_param([model_name, '/Brake_Limit_SW'], 'PortHandles');
ph_bgate   = get_param([model_name, '/Brake_Light_Gate'], 'PortHandles');
ph_blight  = get_param([model_name, '/Brake_Light_LED_3W'], 'PortHandles');
ph_strobe  = get_param([model_name, '/Strobe_LED_3W'], 'PortHandles');
ph_disp    = get_param([model_name, '/Dashboard_Display'], 'PortHandles');
ph_in1     = get_param([model_name, '/In_1_Speed'], 'PortHandles');
ph_in2     = get_param([model_name, '/In_2_BattV'], 'PortHandles');
ph_in3     = get_param([model_name, '/In_3_Temp'], 'PortHandles');
ph_in4     = get_param([model_name, '/In_4_Fault'], 'PortHandles');

ph_throttle = get_param([model_name, '/Hall_Throttle_0.8_4.2V'], 'PortHandles');
ph_brake    = get_param([model_name, '/Brake_Pedal'], 'PortHandles');
ph_timer    = get_param([model_name, '/Timer_Module_2s'], 'PortHandles');
ph_rtds     = get_param([model_name, '/RTDS_Sounder_90dB'], 'PortHandles');

% Wire 1: Battery B1 & B2 to Parallel Bus
add_line(model_name, ph_b1.Outport(1), ph_bus.Inport(1));
add_line(model_name, ph_b2.Outport(1), ph_bus.Inport(2));

% Wire 2: Parallel Bus to F1, F2, and Contactor
add_line(model_name, ph_bus.Outport(1), ph_f1.Inport(1));
add_line(model_name, ph_bus.Outport(1), ph_f2.Inport(1));
add_line(model_name, ph_bus.Outport(1), ph_cont.Inport(1));

% Wire 3: Contactor Coil Trigger & Reference
add_line(model_name, ph_coil.Outport(1), ph_cont.Inport(2));
add_line(model_name, ph_hv0.Outport(1), ph_cont.Inport(3));

% Wire 4: Contactor & F2 to COMPAGE Controller (+72V & -72V)
add_line(model_name, ph_cont.Outport(1), ph_mc.Inport(1));
add_line(model_name, ph_f2.Outport(1), ph_mc.Inport(2));

% Wire 5: Controller 3-Phase outputs to COMPAGE 6kW PMSM Motor
add_line(model_name, ph_mc.Outport(1), ph_motor.Inport(1));
add_line(model_name, ph_mc.Outport(2), ph_motor.Inport(2));
add_line(model_name, ph_mc.Outport(3), ph_motor.Inport(3));

% Wire 6: 12V Safety Shutdown Chain
add_line(model_name, ph_aux.Outport(1), ph_safety.Inport(1));
add_line(model_name, ph_k1.Outport(1), ph_safety.Inport(2));
add_line(model_name, ph_key.Outport(1), ph_safety.Inport(3));
add_line(model_name, ph_k2.Outport(1), ph_safety.Inport(4));
add_line(model_name, ph_tog.Outport(1), ph_safety.Inport(5));

% Wire 7: Kill SW1 Tap
add_line(model_name, ph_aux.Outport(1), ph_k1tap.Inport(1));
add_line(model_name, ph_k1.Outport(1), ph_k1tap.Inport(2));

% Wire 8: Aux Loads (Brake Light, Strobe, Display Power)
add_line(model_name, ph_k1tap.Outport(1), ph_bgate.Inport(1));
add_line(model_name, ph_bsw.Outport(1), ph_bgate.Inport(2));
add_line(model_name, ph_bgate.Outport(1), ph_blight.Inport(1));
add_line(model_name, ph_k1tap.Outport(1), ph_strobe.Inport(1));
add_line(model_name, ph_k1tap.Outport(1), ph_disp.Inport(1));

% Wire 9: Display In 1..4 Sensor Inputs
add_line(model_name, ph_in1.Outport(1), ph_disp.Inport(2));
add_line(model_name, ph_in2.Outport(1), ph_disp.Inport(3));
add_line(model_name, ph_in3.Outport(1), ph_disp.Inport(4));
add_line(model_name, ph_in4.Outport(1), ph_disp.Inport(5));

% Wire 10: Safety Loop Output to Contactor Coil and Timer Module
add_line(model_name, ph_safety.Outport(1), ph_coil.Inport(1));
add_line(model_name, ph_safety.Outport(1), ph_timer.Inport(1));
add_line(model_name, ph_timer.Outport(1), ph_rtds.Inport(1));

% Wire 11: Driver Pedals to Motor Controller
add_line(model_name, ph_throttle.Outport(1), ph_mc.Inport(3));
add_line(model_name, ph_brake.Outport(1), ph_mc.Inport(4));

% Auto-fit zoom to entire diagram
set_param(model_name, 'ZoomFactor', 'FitSystem');

try
    save_system(model_name);
    disp(['  Model Saved: ', model_name, '.slx']);
catch
    disp('  Model generated and open on canvas.');
end

disp('=================================================================');
disp('  DYP DPU PEGASUS 180kg / 6kW MODEL (13T/29T) GENERATED!         ');
disp('=================================================================');
