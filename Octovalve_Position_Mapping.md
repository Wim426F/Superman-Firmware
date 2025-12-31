# Octovalve Position Mapping for Thermal Demands

| **Demand**            | **Source/Sink** | **Octovalve Position** | **Heat Pump** | **Concept**                                                                                   |
|-----------------------|-----------------|------------------------|---------------|-------------------------------------------------------------------------------------------------|
| **Cabin Heating**     | Ambient         | Mode 3                 | On            | Radiator absorbs ambient heat, coolant transfers it to evaporator for refrigerant heating.      |
|                       | Powertrain      | Mode 4                 | On            | Series loop uses powertrain heat, coolant warms evaporator for cabin heating.                   |
|                       | Battery         | Mode 5                 | On            | Battery loop to evaporator transfers battery heat to refrigerant for cabin heating.             |
| **Cabin Cooling**     | Ambient         | Mode 5                 | On            | Powertrain loop to radiator rejects heat; battery loop supports evaporator cooling if needed.   |
|                       | Battery         | Mode 5                 | On            | Battery loop to evaporator, heat rejected to battery via condenser and coolant.                 |
| **Battery Heating**   | Powertrain      | Mode 4                 | Optional      | Series loop transfers powertrain heat to battery, bypassing radiator.                          |
|                       | Ambient         | Mode 3                 | On            | Radiator heat to coolant, then to battery via condenser and pump.                               |
| **Battery Cooling**   | Ambient         | Mode 2                 | Off           | Series loop with radiator dissipates battery heat to ambient air.                              |
|                       | Cabin           | Mode 5                 | On            | Battery heat to evaporator, rejected via refrigerant to cabin or ambient.                       |
| **Powertrain Cooling**| Ambient         | Mode 5                 | Off           | Parallel loop connects powertrain to radiator for heat rejection.                              |
|                       | Battery         | Mode 4                 | Optional      | Series loop transfers powertrain heat to battery if cooler, though ambient is preferred.        |

## Mode Descriptions

- **Mode 2: Series Cooling**  
  This mode connects the battery and powertrain in a single coolant loop that includes the radiator, primarily for cooling both components simultaneously. 

- **Mode 3: Ambient Heating**  
  This mode uses ambient air as a heat source to warm the cabin or battery. The battery and powertrain are in series.
  
- **Mode 4: Heat Scavenge Heating**  
  This position connects the battery and powertrain in series, bypassing the radiator to use only internal heat for cabin or battery heating. 
  
- **Mode 5: Parallel Cooling**  
  This mode separates the battery and powertrain into independent coolant loops. Battery actively cooled, powertrain passively cooled