# Electric Kart Powertrain Model (Team Pegasus)

MATLAB, Simulink and LTspice files for a 72 V, 6 kW electric kart, made for FKDC Season 10 (EV category) by Team The Pegasus (K2E05), Dr. D. Y. Patil Institute of Technology.

I was in the team's Electrical & Electronics department. My part was the Simulink block model and the 12 V safety loop.

**The project is at design stage. Every number here is calculated or simulated, and none was measured on a kart or a track.**

## These are not the final design numbers

These files are from August 2026 and use 13T/29T gearing (2.23:1). Our final design report from October 2026 moved to 3:1 gearing with a 60 km/h design top speed. So the lap results below do not describe the final design.

## Values used in these files

| Parameter | Value |
|---|---|
| Battery | 72 V, 84 Ah (two 20S14P NMC packs of 42 Ah in parallel), about 6.05 kWh |
| Motor | COMPAGE PMSM, 6 kW rated, 61 N·m peak |
| Gearing | 13T/29T chain, 2.23:1 |
| Mass | 180 kg with driver |
| Rear wheel radius | 0.1397 m |

## Lap model results

These are the values `simulate_kari_lap.m` printed in MATLAB R2025b on 3 Nov 2026, for Kari Motor Speedway, Coimbatore.

| Metric | Value |
|---|---|
| Lap length used | 2,090 m |
| Lap time | 110.40 s |
| Top speed | 118.90 km/h |
| Average speed (distance ÷ time) | 68.2 km/h |
| Peak motor torque | 61.00 N·m (136.08 N·m at the axle) |
| Peak battery current | 131.94 A (65.97 A per pack) |
| Energy per lap | 117.43 Wh |
| Laps on a full pack | 51.5 (107.6 km) |

## Known problems

1. **Average speed print.** The script prints 73.92 km/h because it averages over track points. Distance ÷ time gives 68.2 km/h, so use that.
2. **Lap length.** The track is listed as 2.1 km, but the 15 sectors in the script add up to 2,090 m.
3. **The Simulink model is simple.** It is a signal-level block model (55 blocks), not a full electrical simulation. For example, the battery packs are constant blocks. It loads and runs in R2025b, and I checked it with a 1 s run.
4. **LTspice.** `ev_powertrain.cir` runs in batch mode on LTspice 17.2.4 for Mac, but I only checked that it runs, not that its waveforms are right. It also writes a result file of about 390 MB. I could not test the `.asc` schematic in batch mode.

## Files

| File | What it is |
|---|---|
| `ev_gokart_params.m` | All the values above |
| `simulate_kari_lap.m` | Lap time, speed, current and energy over the track |
| `create_ev_gokart_simulink.m` | Builds the Simulink block model |
| `ev_powertrain_gokart.slx` | The Simulink block model |
| `ev_powertrain.cir` | LTspice netlist of the power path and 12 V safety loop |
| `ev_powertrain.asc` | LTspice schematic |

## Run it

In MATLAB, from this folder:

```matlab
simulate_kari_lap
create_ev_gokart_simulink
```

`create_ev_gokart_simulink` rebuilds `ev_powertrain_gokart.slx` and overwrites the copy in this folder.
