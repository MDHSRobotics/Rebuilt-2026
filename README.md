# Rebuilt-2026
A repository for the Rebuilt 2026 FRC game season.

# FRC Team 4141 — Artiller-E 2026

> Autonomous-capable double shooter bot for the 2026 REBUILT game.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Software Architecture](#software-architecture)
- [Subsystems](#subsystems)
- [Commands](#commands)
- [Autonomous Modes](#autonomous-modes)
- [Controls](#controls)
- [Getting Started](#getting-started)
- [Building & Deploying](#building--deploying)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

| Field         | Details                         |
|---------------|---------------------------------|
| Team Number   | 4141                            |
| Robot Name    | Artiller-E                      |
| Season / Game | 2026 — REBUILT                  |
| Language      | Java                            |
| Framework     | WPILib Command-Based            |
| Build System  | Gradle                          |

---

## Hardware

### Drive Train
- **Type:** Swerve
- **Motors:** 8x Falcon 500
- **Motor Controllers:** TalonFx
- **Gyro:** Pigeon 2

### Other Mechanisms
| Mechanism        | Motors           | Controller    | Sensors                  |
|------------------|------------------|---------------|--------------------------|
| Shooter          | NEO Vortex x 2   | SPARK Flex    | None                     |
| Kicker           | NEO Vortex x 1   | SPARK Flex    | None                     |
| Hopper           | NEO Vortex x 1   | SPARK Flex    | None                     |
| Intake           | NEO Vortex x 3   | SPARK Flex    | Through-bore encoder x 2 |

### CAN IDs
| Device              | CAN ID |
|---------------------|--------|
| Gyro (Pigeon 2)     | 0      |
| Front Left Drive    | 8      |
| Front Left Turn     | 9      |
| Front Right Drive   | 10     |
| Front Right Turn    | 11     |
| Rear Left Drive     | 12     |
| Rear Left Turn      | 13     |
| Rear Right Drive    | 14     |
| Rear Right Turn     | 15     |
| Intake Right        | 16     |
| Intake Left         | 17     |
| Intake Spinner      | 18     |
| Shooter Right       | 19     |
| Shooter Left        | 20     |
| Kicker              | 21     |
| Hopper              | 22     |

---

## Software Architecture

This robot uses the **WPILib Command-Based** framework.

```
src/main/java/frc/robot/
├── Main.java
├── Robot.java                  # Entry point, lifecycle hooks
├── RobotContainer.java         # Subsystem & command wiring, button bindings
├── Constants.java              # All hardware constants and tuning values
├── commands/
│   ├── [ExampleCommand].java
│   └── autos/
│       └── [ExampleAuto].java
└── subsystems/
    ├── CommandSwerveDrivetrain.java
    ├── Shooter.java
    ├── Hopper.java
    └── Intake.java
    
```

---

## Subsystems

### DriveSubsystem
- **Description:** Controls robot movement using swerve.
- **Default Command:** `DefaultDriveCommand` — joystick field-relative drive.
- **Key Methods:**
  - `drive(double xSpeed, double ySpeed, double rot)` — drives the robot.
  - `resetOdometry(Pose2d pose)` — resets pose estimate.
  - `getPose()` — returns current `Pose2d`.

### ShooterSubystem
- **Description:** Controls the arm for [game piece] manipulation.
- **Default Command:** Stops all motors.
- **Key Methods:**
  - `setAngle(double degrees)` — moves arm to target angle.
  - `getAngle()` — returns current arm angle.

### HopperSubystem
- **Description:** Controls the rollers on the hopper to feed the shooter.
- **Default Command:** Stops all motors.
- **Key Methods:**
  - `setAngle(double degrees)` — moves arm to target angle.
  - `getAngle()` — returns current arm angle.

### IntakeSubsystem
- **Description:** Intakes game pieces.
- **Default Command:** Stops spinner and returns to stowed position.
- **Key Methods:**
  - `intake()` — runs intake inward.
  - `outtake()` — runs intake outward.
  - `stop()` — stops intake.

> _Add additional subsystems as needed._

---

## Commands

| Command                  | Description                                            | Used By               |
|--------------------------|--------------------------------------------------------|-----------------------|
| `DefaultDriveCommand`    | Joystick-driven teleop driving                         | DriveSubsystem        |
| `IntakeCommand`          | Runs intake while held                                 | Operator controller   |
| `ArmToPositionCommand`   | Moves arm to a preset angle                            | Operator controller   |
| `AutoBalanceCommand`     | Auto-levels on the charging station                    | Auto / teleop         |
| `DriveToTargetCommand`   | Vision-assisted alignment to a target                  | Auto / teleop         |

> _Add or remove commands to match your robot._

---

## Autonomous Modes

Autonomous routines are selected via **Shuffleboard / SmartDashboard** using a `SendableChooser`.

| Mode Name              | Description                                          |
|------------------------|------------------------------------------------------|
| `DoNothingAuto`        | Robot does not move (default/fallback).              |
| `TaxiAuto`             | Drives out of the starting zone.                     |
| `OnePieceAuto`         | Scores one preloaded game piece and taxis.           |
| `TwoPieceAuto`         | Scores preload, intakes, and scores a second piece.  |
| `[CustomAuto]`         | [Description]                                        |

Autos are implemented using **WPILib's Command Composition** and/or **PathPlanner**.

---

## Controls

### Driver — Controller Port 0 PS4

| Input              | Action                              |
|--------------------|-------------------------------------|
| Left Stick X/Y     | Translate (field-relative)          |
| Right Stick X      | Rotate                              |
| Left Bumper        | Slow mode (50% speed)               |
| Right Bumper       | Turbo mode (100% speed)             |
| Back Button        | Reset field-relative heading        |
| Start Button       | Reset odometry                      |
| A Button           | Auto-align to target                |

### Operator — Controller Port 1 Xbox

| Input              | Action                              |
|--------------------|-------------------------------------|
| Left Stick Y       | Manual arm control                  |
| Right Trigger      | Intake                              |
| Left Trigger       | Outtake                             |
| A Button           | Arm to floor position               |
| B Button           | Arm to mid position                 |
| Y Button           | Arm to high position                |
| X Button           | Arm to stow position                |
| Left Bumper        | Climber up                          |
| Right Bumper       | Climber down                        |

---

## Getting Started

### Prerequisites

- [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) installed
- [FRC Game Tools](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html) installed
- Java 17+
- Git

### Clone the Repository

```bash
git clone https://github.com/[ORG]/[REPO_NAME].git
cd [REPO_NAME]
```

### Open in VS Code

1. Open VS Code.
2. Open the cloned folder: **File → Open Folder**.
3. VS Code should detect the WPILib project automatically.

---

## Building & Deploying

### Build (Simulation / Check)
```bash
./gradlew build
```

### Deploy to Robot
Ensure the computer is connected to the robot's network (radio or tether), then:
```bash
./gradlew deploy
```

### Run Simulation
```bash
./gradlew simulateJava
```

---

## Contributing

1. Fork the repo and create a feature branch: `git checkout -b feature/my-feature`
2. Commit your changes with clear messages: `git commit -m "Add: arm position presets"`
3. Push to your branch: `git push origin feature/my-feature`
4. Open a Pull Request and request review from a lead.

### Code Style
- Follow WPILib Java conventions.
- All constants go in `Constants.java`.
- Every subsystem and command should have a Javadoc summary comment.
- Avoid `magic numbers` — use named constants.

---

## License

This project is licensed under the [MIT License](LICENSE) — see the `LICENSE` file for details.

---

*Go Monarch Robotics!*