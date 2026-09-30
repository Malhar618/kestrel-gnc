# Architecture

Two halves that never include each other:

- **`fsw/`** is the flight software: the code that would fly. Float, no heap, no exceptions,
  fixed-rate, deterministic.
- **`sim/`** is the world: 6-DOF dynamics, sensor models and wind, in double precision.

They only meet through plain message structs: in-process first, later over MAVLink.

## Guidance, navigation and control loop

```mermaid
flowchart LR
  subgraph FSW["fsw/ · flight software (float, no heap)"]
    G["Guidance<br/>min-snap trajectory"] -->|"pos/vel/acc setpoint"| C["Control<br/>position → attitude → rate<br/>PID / LQR"]
    C -->|"body torque + thrust"| MX["Mixer<br/>→ motor commands 0–1"]
    N["Navigation<br/>error-state EKF"] -->|"state estimate"| G
    N -->|"state estimate"| C
  end
  subgraph SIM["sim/ · the world (double)"]
    P["6-DOF plant<br/>rigid body + rotors + drag"]
    E["Environment<br/>gravity, wind, gusts"] --> P
    S["Sensor models<br/>IMU · GPS · baro"]
    P -->|"true state"| S
  end
  MX -->|"ActuatorCmd"| P
  S -->|"SensorFrame (timestamped)"| N
```

## One flight-software core, four environments

The same `fsw/core` library runs everywhere. Only the transport under the HAL changes.

```mermaid
flowchart TB
  CORE["fsw/core<br/>GNC library"] --> HAL["HAL: Clock · SensorSource · ActuatorSink · Link"]
  HAL --> A["In-process sim<br/>(unit + Monte Carlo)"]
  HAL --> B["SIL<br/>MAVLink over UDP"]
  HAL --> D["PIL<br/>STM32F4 firmware in Renode,<br/>MAVLink over emulated UART"]
  HAL --> F["Hardware<br/>Jetson (MAVLink to PX4) · real MCU over USB serial"]
```

Milestones and the full plan: see the roadmap in the [README](../README.md).
