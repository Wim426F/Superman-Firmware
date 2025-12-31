# Refrigerant System Valve Settings Overview

## Valve Roles
- **EXPV before Condensers**: Regulates flow with minimal pressure drop to maintain high pressure for heat rejection.
- **EXPV before Evaporators**: Controls evaporation for heat absorption.
- **2/2 Solenoid**: On/off valve for to the coolant condenser, reduces restriction when max flow is needed.

## Settings by Mode
### 1. Cabin Heating
- **EXPV Cabin Condenser**: 100% (minimal restriction)
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 0% (closed)
- **EXPV Cabin Evaporator**: 0% (closed)
- **2/2 Solenoid**: Closed
- **Description**: Uses cabin condenser to heat cabin air, with coolant evaporator absorbing heat from coolant (e.g., from powertrain or ambient). Solenoid is closed as high flow is not needed.

### 2. Cabin Cooling
- **EXPV Cabin Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 0% (closed)
- **EXPV Cabin Condenser**: 0% (closed)
- **EXPV Coolant Evaporator**: 0% (closed)
- **2/2 Solenoid**: Open (high flow)
- **Description**: Cools cabin via cabin evaporator, with coolant condenser rejecting heat to coolant. 2/2 solenoid for coolant condensor opens for minimal restriction.

### 3. Battery/Powertrain Cooling
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 0% (closed)
- **EXPV Cabin Condenser**: 0% (closed)
- **EXPV Cabin Evaporator**: 0% (closed)
- **2/2 Solenoid**: Open (high flow)
- **Description**: Cools battery/powertrain via coolant evaporator, with coolant condenser rejecting heat. 2/2 solenoid for coolant condensor opens for minimal restriction.

### 4. Battery/Powertrain Heating
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 0% (closed)  
- **EXPV Cabin Condenser**: 0% (closed)
- **EXPV Cabin Evaporator**: 0% (closed)
- **2/2 Solenoid**: Open (high flow)
- **Description**: Heats battery/powertrain by rejecting heat to coolant via coolant condenser, with coolant evaporator absorbing heat (e.g., from ambient). The 2/2 solenoid is opened to maximize refrigerant flow to the coolant condenser, as only this condenser is in use. The EXPV is closed, reserved for modes requiring flow splitting between multiple condensers.

### 5. Dual Mode: Cabin Heating + Battery Cooling
- **EXPV Cabin Condenser**: 50% (balanced flow)
- **EXPV Coolant Condenser**: 50% (balanced flow)
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Cabin Evaporator**: 0% (closed)
- **2/2 Solenoid**: Closed
- **Description**: Heats cabin via cabin condenser while cooling battery via coolant evaporator. EXPV balance flow between condensers; solenoid closed to avoid flow imbalance.

### 6. Dual Mode: Cabin Heating + Battery Heating
- **EXPV Cabin Condenser**: 50% (balanced flow)
- **EXPV Coolant Condenser**: 50% (balanced flow)
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Cabin Evaporator**: 0% (closed)
- **2/2 Solenoid**: Closed
- **Description**: Heats both cabin and battery via cabin and coolant condensers. Coolant evaporator absorbs heat (e.g., from ambient). Solenoid closed for balanced flow.

### 7. Dual Mode: Cabin Cooling + Battery Cooling
- **EXPV Cabin Evaporator**: controlled evaporation
- **EXPV Coolant Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 0% (closed)
- **EXPV Cabin Condenser**: 0% (closed)
- **2/2 Solenoid**: Open (high flow)
- **Description**: Cools both cabin and battery via respective evaporators, with coolant condenser rejecting heat. 2/2 solenoid for coolant condensor opens for minimal restriction.

### 8. Dual Mode: Cabin Cooling + Battery Heating
- **EXPV Cabin Evaporator**: controlled evaporation
- **EXPV Coolant Condenser**: 100% (minimal restriction)
- **EXPV Cabin Condenser**: 0% (closed)
- **EXPV Coolant Evaporator**: 0% (closed)
- **2/2 Solenoid**: Closed
- **Description**: Cools cabin via cabin evaporator while heating battery via coolant condenser. Heat from cabin is transferred to battery coolant; solenoid closed for controlled flow.

## Notes
- **2/2 Solenoid**: Used for high-flow modes where only one condenser is active (e.g., Modes 2, 3, 4, 7); closed in dual modes requiring flow splitting (e.g., Modes 5, 6, 8).
- **EXPV Settings**: Percentages are approximate and dynamically adjusted based on thermal load, superheat, or temperature feedback.
- **Flow Balance**: In dual modes with multiple condensers (e.g., Modes 5, 6), EXPV settings ensure balanced refrigerant distribution.