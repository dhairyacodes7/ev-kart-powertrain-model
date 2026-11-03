%% =========================================================================
%% Dr. D. Y. Patil Institute of Technology (DYP DPU) - TEAM PEGASUS
%% KARI MOTOR SPEEDWAY (2.1 km) - EXACT LAP TIME & ENERGY SIMULATOR
%% =========================================================================
% Solves the optimal velocity profile along the full 2100m race track using:
% - Vehicle Mass = 180 kg (Kart + Driver)
% - COMPAGE 6 kW PMSM Motor (19 N*m rated, 61 N*m peak torque)
% - 420 Chain Drivetrain: 13T Pinion / 29T Sprocket (GR = 2.23077)
% - Rear Tyres: 11-inch (Radius r = 0.1397 m), Front: 10-inch (Radius r = 0.127 m)
% - Resistances: Crr = 0.01 (Frr = 17.65N), Cd = 0.9, A = 0.20m^2, rho = 1.2 kg/m^3
% - Braking: mu = 0.85 (Fbrake = 1500 N)
% - 2x 72V 42Ah 3C NMC Battery Packs (72V 84Ah, 6.05 kWh total capacity)

clear; clc; close all;

%% Load System Specs
ev_gokart_params;

%% 1. Define Kari Motor Speedway Circuit Geometry (2100 m, 15 Turns)
track_sectors = [
    struct('name', 'Sector 1: Front Straight 1',  'length', 290.0, 'R', Inf,  'desc', 'Launch / Main Straight')
    struct('name', 'Sector 2: Turn 1 Hairpin',    'length',  85.0, 'R', 18.0, 'desc', 'Heavy Braking Hairpin')
    struct('name', 'Sector 3: Chute to T2',       'length',  60.0, 'R', Inf,  'desc', 'Short Acceleration')
    struct('name', 'Sector 4: Turn 2-3 Chicane',  'length',  95.0, 'R', 22.0, 'desc', 'Technical Left-Right')
    struct('name', 'Sector 5: Infield Chute',     'length',  70.0, 'R', Inf,  'desc', 'Infield Straight')
    struct('name', 'Sector 6: Turn 4-6 Complex',  'length', 120.0, 'R', 26.0, 'desc', 'Tight Infield Loop')
    struct('name', 'Sector 7: Front Straight 2',  'length', 360.0, 'R', Inf,  'desc', 'Longest Straight (Top Speed)')
    struct('name', 'Sector 8: Turn 7 Chicane',    'length',  75.0, 'R', 16.0, 'desc', 'Sharp Left Chicane')
    struct('name', 'Sector 9: Mid Straight',      'length',  80.0, 'R', Inf,  'desc', 'Mid Acceleration')
    struct('name', 'Sector 10: Turns 8-10 S-Curves','length',160.0,'R', 30.0, 'desc', 'Flowing High-Speed S')
    struct('name', 'Sector 11: Connecting Chute', 'length',  70.0, 'R', Inf,  'desc', 'Bridge Chute')
    struct('name', 'Sector 12: Turns 11-12 Sweeper','length',115.0,'R', 36.0, 'desc', 'Fast Right Sweeper')
    struct('name', 'Sector 13: Back Straight',    'length', 250.0, 'R', Inf,  'desc', 'Full Power Back Straight')
    struct('name', 'Sector 14: Turn 13-14 Complex','length',130.0, 'R', 20.0, 'desc', 'Penultimate Complex')
    struct('name', 'Sector 15: Turn 15 & Finish', 'length', 130.0, 'R', 28.0, 'desc', 'Final Turn to Finish Line')
];

%% 2. Discretize Track into Spatial Grid
ds = 0.5; % 0.5 meter spatial resolution
s_vec = [];
v_corner_limit = [];
sector_id = [];

mu_lat = 1.05; % Lateral tire friction on tarmac
g = params.Kart.g;
mass = params.Kart.mass;
r_wheel = params.Kart.wheel_radius_rear;
GR = params.Kart.gear_ratio;

for k = 1:length(track_sectors)
    n_pts = round(track_sectors(k).length / ds);
    R_val = track_sectors(k).R;
    if isinf(R_val)
        v_lim = 120.0 / 3.6; % Theoretical max gearing limit (120 km/h)
    else
        v_lim = sqrt(mu_lat * g * R_val); % Apex cornering speed limit
    end
    s_vec = [s_vec; repmat(ds, n_pts, 1)];
    v_corner_limit = [v_corner_limit; repmat(v_lim, n_pts, 1)];
    sector_id = [sector_id; repmat(k, n_pts, 1)];
end

N = length(s_vec);
dist_track = cumsum(s_vec);
total_track_length = dist_track(end);

%% 3. Two-Pass Dynamic Velocity Profiler (Forward Accel + Backward Braking)
% Forward Pass (Full Traction & Aero Drag)
v_fwd = zeros(N, 1);
v_fwd(1) = 0.0; % Standing Start from grid (or set to 15.0 for Flying Lap)

for i = 1:(N-1)
    v_curr = v_fwd(i);
    w_mot = (v_curr / r_wheel) * GR;
    
    % Motor Torque & Power Envelope (61 Nm Peak, 9 kW Peak Power)
    if w_mot < 1.0
        T_mot = params.Motor.T_peak;
    else
        T_mot = min(params.Motor.T_peak, 9000.0 / w_mot);
    end
    
    F_trac = (T_mot * GR) / r_wheel;
    F_aero = 0.5 * params.Kart.rho_air * params.Kart.Cd * params.Kart.frontal_area * (v_curr^2);
    F_roll = params.Kart.C_rr * mass * g;
    F_net = max(0, F_trac - F_aero - F_roll);
    a_accel = F_net / mass;
    
    v_next = sqrt(v_curr^2 + 2 * a_accel * ds);
    v_fwd(i+1) = min(v_next, v_corner_limit(i+1));
end

% Backward Pass (Full Braking Force)
v_bwd = zeros(N, 1);
v_bwd(end) = v_corner_limit(end);
F_brake_max = 0.85 * mass * g; % 1500 N braking force

for i = N:-1:2
    v_curr = v_bwd(i);
    F_aero = 0.5 * params.Kart.rho_air * params.Kart.Cd * params.Kart.frontal_area * (v_curr^2);
    F_roll = params.Kart.C_rr * mass * g;
    F_decel = F_brake_max + F_aero + F_roll;
    a_decel = F_decel / mass;
    
    v_prev = sqrt(v_curr^2 + 2 * a_decel * ds);
    v_bwd(i-1) = min(v_prev, v_corner_limit(i-1));
end

% Optimal Lap Velocity Profile (Intersection of Accel & Decel Limits)
v_lap = min(v_fwd, v_bwd);
v_lap_kmh = v_lap * 3.6;

%% 4. Calculate Time, Power, Current & Energy
dt_segment = ds ./ max(0.5, (v_lap(1:end-1) + v_lap(2:end)) / 2.0);
time_lap = [0; cumsum(dt_segment)];
total_lap_time = time_lap(end);

% Power & Energy Profile
p_elec_kw = zeros(N, 1);
i_pack_a = zeros(N, 1);
motor_torque_profile = zeros(N, 1);

for i = 1:N
    v_curr = v_lap(i);
    w_mot = (v_curr / r_wheel) * GR;
    F_aero = 0.5 * params.Kart.rho_air * params.Kart.Cd * params.Kart.frontal_area * (v_curr^2);
    F_roll = params.Kart.C_rr * mass * g;
    
    % Mechanical power required
    if i < N
        a_local = (v_lap(i+1)^2 - v_lap(i)^2) / (2 * ds);
    else
        a_local = 0.0;
    end
    
    if a_local > 0
        F_drive = mass * a_local + F_aero + F_roll;
        T_wheel = F_drive * r_wheel;
        motor_torque_profile(i) = min(params.Motor.T_peak, T_wheel / GR);
        p_mech = F_drive * v_curr;
        p_elec_kw(i) = min(9.5, (p_mech / 0.92 + params.MC.P_idle) / 1000.0);
    else
        motor_torque_profile(i) = 0.0;
        p_elec_kw(i) = 0.012; % Idle electronics
    end
    i_pack_a(i) = (p_elec_kw(i) * 1000.0) / params.HV.V_nom;
end

% Energy consumption
cum_energy_joules = cumsum([0; p_elec_kw(1:end-1) * 1000.0 .* dt_segment]);
energy_wh_lap = cum_energy_joules(end) / 3600.0;
pack_total_energy_wh = params.HV.V_nom * params.HV.Cap_Ah_total; % 72V * 84Ah = 6048 Wh
total_laps_possible = pack_total_energy_wh / energy_wh_lap;
total_range_km = total_laps_possible * (total_track_length / 1000.0);

%% 5. Display Sector-by-Sector Breakdown
disp('=================================================================');
disp('  KARI MOTOR SPEEDWAY (2.1 km) - SECTOR-BY-SECTOR RACE BREAKDOWN ');
disp('=================================================================');
fprintf('%-30s | %-8s | %-10s | %-10s\n', 'Sector Name', 'Length', 'Apex Speed', 'Sector Time');
disp('-----------------------------------------------------------------');

for k = 1:length(track_sectors)
    mask = (sector_id == k);
    sec_v_max = max(v_lap_kmh(mask));
    sec_v_min = min(v_lap_kmh(mask));
    sec_indices = find(mask);
    if ~isempty(sec_indices) && sec_indices(1) > 1
        t_sec = time_lap(sec_indices(end)) - time_lap(sec_indices(1)-1);
    else
        t_sec = time_lap(sec_indices(end));
    end
    fprintf('%-30s | %5.1f m  | %5.1f km/h | %5.2f s\n', ...
        track_sectors(k).name, track_sectors(k).length, sec_v_max, t_sec);
end

disp('=================================================================');
disp('  OVERALL LAP PERFORMANCE SUMMARY                                ');
disp('=================================================================');
fprintf('Track Distance:        %.1f meters (2.1 km)\n', total_track_length);
fprintf('Total Lap Time:        %.2f seconds (%d min %.2f sec)\n', ...
    total_lap_time, floor(total_lap_time/60), rem(total_lap_time, 60));
fprintf('Top Speed Achieved:    %.2f km/h (Front Straight 2)\n', max(v_lap_kmh));
fprintf('Average Lap Speed:     %.2f km/h\n', mean(v_lap_kmh));
fprintf('Peak Motor Torque:     %.2f N*m (Peak Axle Torque: %.2f N*m)\n', ...
    max(motor_torque_profile), max(motor_torque_profile) * GR);
fprintf('Peak Traction Current: %.2f A (%.2f A per 42Ah pack)\n', max(i_pack_a), max(i_pack_a)/2);
fprintf('Energy Consumed/Lap:   %.2f Wh (%.4f kWh)\n', energy_wh_lap, energy_wh_lap/1000.0);
fprintf('Total Laps on 84Ah:    %.1f full race laps\n', total_laps_possible);
fprintf('Total Track Range:     %.1f km\n', total_range_km);
disp('=================================================================');

%% 6. Diagnostic Visualization (Velocity vs Distance & Time)
figure('Name', 'DYP DPU PEGASUS - Kari Motor Speedway Lap Telemetry', 'Color', 'w', 'Position', [80, 60, 1280, 850]);

subplot(3, 2, 1);
plot(dist_track, v_lap_kmh, 'm-', 'LineWidth', 2); hold on;
plot(dist_track, v_corner_limit * 3.6, 'k--', 'LineWidth', 1.0);
grid on; xlabel('Track Distance (m)'); ylabel('Speed (km/h)');
title('1. Optimal Velocity Profile vs Track Distance (2100m)');
legend('Actual Kart Speed', 'Cornering Grip Limit', 'Location', 'southwest');

subplot(3, 2, 2);
plot(time_lap, v_lap_kmh, 'm-', 'LineWidth', 2);
grid on; xlabel('Time (seconds)'); ylabel('Speed (km/h)');
title(sprintf('2. Kart Speed vs Lap Time (Total: %.2f s / 1m %.1fs)', total_lap_time, rem(total_lap_time, 60)));

subplot(3, 2, 3);
plot(dist_track, p_elec_kw, 'r-', 'LineWidth', 1.8);
grid on; xlabel('Track Distance (m)'); ylabel('Power (kW)');
title('3. Traction Electric Power Demand (kW)'); ylim([0, 11]);

subplot(3, 2, 4);
plot(dist_track, motor_torque_profile, 'b-', 'LineWidth', 1.8); hold on;
plot(dist_track, motor_torque_profile * GR, 'k--', 'LineWidth', 1.2);
grid on; xlabel('Track Distance (m)'); ylabel('Torque (N\cdotm)');
title('4. Motor & Axle Torque Demand');
legend('Motor Torque', 'Axle Torque (13T/29T)', 'Location', 'northeast');

subplot(3, 2, 5);
plot(dist_track, i_pack_a, 'k-', 'LineWidth', 1.8); hold on;
plot(dist_track, i_pack_a / 2, 'b--', 'LineWidth', 1.2);
grid on; xlabel('Track Distance (m)'); ylabel('Current (A)');
title('5. Total & Per-Pack Battery Current Sharing');
legend('Total Current', 'Per-Pack (42Ah)', 'Location', 'northeast');

subplot(3, 2, 6);
plot(dist_track, cum_energy_joules / 3600.0, 'g-', 'LineWidth', 2);
grid on; xlabel('Track Distance (m)'); ylabel('Energy (Wh)');
title(sprintf('6. Cumulative Energy Consumption (%.1f Wh/lap | %.1f Laps Total)', energy_wh_lap, total_laps_possible));
