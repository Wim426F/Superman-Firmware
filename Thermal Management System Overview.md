# Thermal Management System: Core Principles

## For Heating Demands (e.g., Cabin or Battery Heating)
- **Objective**: Select the most efficient heat source available.
- **Key Metric**: Delta T = `target_temp - source_temp`
- **Selection**: Choose the hottest available source (the smallest delta T), minimizing energy needed for heating.
- **Available Sources**:
  - **Powertrain**: Always available (constant heat generation).
  - **Battery**: Available if not too cold.
  - **Ambient**: Always available.
  - **Recirculation**: Always available. Recirculation mode for compressor COP=1

## For Cooling Demands (e.g., Powertrain, Battery, or Cabin Cooling)
- **Objective**: Select the most efficient heat sink available.
- **Key Metric**: Delta T = `target_temp - sink_temp`
- **Selection**: Choose the coldest available sink (the largest delta T), maximizing the temperature gap for efficient heat rejection.
- **Available Sinks**:
  - **Battery**: Available if not too hot.
  - **Ambient**: Always available.


## Example Scenarios
### Heating the Cabin
- **Sources**: Powertrain (30°C), Battery (15°C, if available), Ambient (5°C).
- **Target**: Cabin at 20°C.
- **Delta T**: Powertrain = 20 - 30 = -10°C, Battery = 20 - 15 = 5°C, Ambient = 20 - 5 = 15°C.
- **Choice**: Powertrain (smallest delta T, -10°C, hottest source).

### Cooling the Powertrain
- **Sinks**: Battery (10°C, if available), Ambient (25°C).
- **Target**: Powertrain at 35°C.
- **Delta T**: Battery = 35 - 10 = 25°C delta , Ambient = 35 - 25 = 10°C delta.
- **Choice**: Battery (largest delta T, 25°C, coldest sink).
