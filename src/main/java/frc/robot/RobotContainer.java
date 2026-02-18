// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;

public class RobotContainer {
  // Robot Speed from 0% to 100%
  private double m_robotSpeed = 1.0 * Constants.MAX_LINEAR_SPEED;

  private double m_angularVelocity = Constants.MAX_ANGULAR_VELOCITY;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(getDeadband())
          .withRotationalDeadband(getRotationalDeadband()) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final DriveTelemetry m_logger = new DriveTelemetry(m_robotSpeed);

  /* Controllers  */
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(Constants.ControllerConstants.DRIVER_CONTROLLER_PORT);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  public RobotContainer() {
    setDefaultCommands();
    configureDriverControllers();
  }

  private void setDefaultCommands() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(-m_driverController.getLeftY() * m_robotSpeed) // Drive
                    // forward
                    // with
                    // negative Y
                    // (forward)
                    .withVelocityY(
                        -m_driverController.getLeftX()
                            * m_robotSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -m_driverController.getRightX()
                            * m_angularVelocity) // Drive counterclockwise
            // with negative X (left)
            ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.triangle().whileTrue(m_drivetrain.applyRequest(() -> brake));
    m_driverController
        .circle()
        .whileTrue(
            m_drivetrain.applyRequest(
                () ->
                    point.withModuleDirection(
                        new Rotation2d(
                            -m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController
    //     .share()
    //     .and(m_driverController.triangle())
    //     .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController
    //     .share()
    //     .and(m_driverController.square())
    //     .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController
    //     .options()
    //     .and(m_driverController.triangle())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController
    //     .options()
    //     .and(m_driverController.square())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    m_driverController.options().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  /** Use this method to map driver controls and commands */
  private void configureDriverControllers() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController
    //     .share()
    //     .and(m_driverController.triangle())
    //     .whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController
    //     .share()
    //     .and(m_driverController.square())
    //     .whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController
    //     .options()
    //     .and(m_driverController.triangle())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController
    //     .options()
    //     .and(m_driverController.square())
    //     .whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Quarter Speed
    m_driverController.R2().onTrue(Commands.runOnce(() -> m_robotSpeed = 0.25));

    m_driverController.R2().onFalse(Commands.runOnce(() -> m_robotSpeed = 1.0));
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        m_drivetrain
            .applyRequest(() -> m_drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
            .withTimeout(5.0),
        // Finally idle for the rest of auton
        m_drivetrain.applyRequest(() -> idle));
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The linear deadband in meters per second.
   */
  public double getDeadband() {
    return m_robotSpeed * 0.1;
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The rotational deadband in radians per second.
   */
  public double getRotationalDeadband() {
    return m_angularVelocity * 0.1;
  }
}
