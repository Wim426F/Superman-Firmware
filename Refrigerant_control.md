# Refrigerant control

The refrigerant controller executes at 100 ms in `runCapacityControl()` (`src/thermal_control.cpp`).

## Actuators

The evaporator expansion valve (EXV) is the refrigerant orifice. Opening determines suction pressure and therefore evaporating temperature.

Compressor duty sets available refrigerant mass flow.

Condenser expansion valves are excluded from this controller. They are used only as on/off flow dividers (255 = open, 0 = closed).

Inactive evaporator EXVs are commanded to 0. All active evaporator EXVs receive the same command: suction temperature is measured at one point, so there is a single inner loop.

## Cascade

Two loops.

**Inner loop (EXV).** A velocity-form PI controller sets evaporator EXV position. The process variable depends on mode:

- cooling: compressor inlet temperature versus `temp_evaporator_setp`
- heating: compressor outlet temperature versus `temp_condensor_setp`

Error is measured minus setpoint. Positive error reduces EXV opening; negative error increases it. The sign is the same in both modes.

**Outer loop (compressor).** Compressor duty is adjusted from inner-loop saturation:

| Condition | Duty command |
|---|---|
| Process above setpoint by more than 2 °C and EXV ≤ 50 | Increase |
| Process below setpoint by more than 2 °C and EXV ≥ 250 | Decrease |
| Process within ±2 °C and EXV > 60 | Decrease |
| Process within ±2 °C and EXV ≤ 60 | Hold |
| Discharge pressure above limit or suction pressure below limit | Unload |

Steady state is process temperature within ±2 °C of setpoint and EXV opening near 50. That operating point is the lowest mass flow that maintains the setpoint without reaching the stall limit of 40.

## Initialisation

Plant thermal capacity is unknown. The controller therefore begins at a high compressor duty and reduces it after the inner loop has settled.

On entry to heating or cooling:

- EXV command is set to 100 (`EXPV_START_POS`)
- compressor duty target is set to 65 % (`COMPRESSOR_START_DUTY`)
- compressor duty ramps from 0 % at 2 % per 100 ms
- EXV position is limited to 4 counts per 100 ms

## Limits

| Constant | Value | Function |
|---|---|---|
| `EXPV_MIN_POS` | 40 | Minimum EXV opening while the valve is active (compressor stall) |
| `EXPV_SAT_POS` | 50 | Inner loop treated as fully closed for the outer loop |
| `EXPV_START_POS` | 100 | EXV command at mode entry |
| `COMPRESSOR_START_DUTY` | 65 % | Duty target at mode entry |
| `COMPRESSOR_MIN_RUN` | 15 % | Minimum non-zero duty |
| `COMPRESSOR_TRIM` | 0.5 % / 100 ms | Outer-loop increment |
| `T_HOLD_BAND` | 2 °C | Setpoint tolerance |
| `Kp`, `Ki` | 4, 0.8 | Inner-loop gains; output is then rate-limited |

Duty in (0 %, 15 %) is not used. A target below 15 % is set to 0 %. Re-enable from 0 % is at 15 % when the process is above setpoint and the EXV is at or below 50.

## Parallel evaporators

Enabling a second evaporator drives its EXV to the current inner-loop command (not below 40). Combined orifice area increases and suction temperature rises. The inner loop reduces both openings toward 50. If the process remains above setpoint, the outer loop increases compressor duty.

## Idle

All expansion valves are commanded to 255. Compressor duty is 0 %. The following heating or cooling entry re-initialises the cascade as specified under Initialisation.
