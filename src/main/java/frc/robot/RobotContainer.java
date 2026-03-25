// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AimingCommand;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveTelemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperPowers;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.Aiming;
import frc.robot.util.DynamicAutoCreator;
import frc.robot.util.HubStatus;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Testable;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
  // Robot Speed from 0% to 100%
  private double m_robotSpeed = 1.0;

  private double m_testShooterRPM = 2500;

  /* Setting up bindings for necessary control of the swerve drive platform */
  // private final DriveWithSetpointGeneration m_drive =
  //     new DriveWithSetpointGeneration(
  //             DriveConstants.SWERVE_SETPOINT_GENERATOR, Constants.UPDATE_PERIOD)
  //         .withDriveRequestType(DriveRequestType.Velocity)
  //         .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(getDeadband())
          .withRotationalDeadband(getRotationalDeadband()) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake m_brake =
      new SwerveRequest.SwerveDriveBrake()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Hopper m_hopper = new Hopper();

  // Autonomous Chooser - A set of options for specifying the active autonomous command from a
  // dashboard like Elastic
  private SendableChooser<Command> m_autoChooser;

  /* Autonomous Creator - This dynamically creates commands based on settings in the Elastic Auto tab */
  private final DynamicAutoCreator m_dynamicAutoCreator =
      new DynamicAutoCreator(this::resetFieldPosition, m_shooter);

  private final DriveTelemetry m_logger = new DriveTelemetry(DriveConstants.MAX_LINEAR_SPEED);

  /* Controllers  */
  private final CommandPS4Controller m_driverController =
      new CommandPS4Controller(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  private final AimingCommand m_aimingCommands =
      new AimingCommand(m_drivetrain, this::getVelocityX, this::getVelocityY, this::getDeadband);

  private final List<Testable> testableSubsystems = List.of(m_intake, m_hopper, m_shooter);

  public RobotContainer() {
    setDefaultCommands();
    configureDriverControllers();
    configureOperatorControllers();
    setupAutoCommandOptions();
    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  /* Define the possible auto command options that can be chosen from the dashboard.
   * This includes:
   *  - Pre-defined auto commands from PathPlanner
   *  - Explicitly defined auto commands
   *  - Dynamically generated commands using parameter settings displayed on the dashboard
   *    (such as starting position, paths, and actions)
   */
  private void setupAutoCommandOptions() {

    // First preload any auto commands statically defined in PathPlanner,
    // specifying which of the PathPlanner commands should be the default (if any)
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Explicitly add any other auto commands
    m_autoChooser.addOption("------------------------", Commands.none());
    m_autoChooser.addOption("Print Test", new RunCommand(() -> System.out.println("Test")));

    // Publish the auto command chooser to the dashboard
    SmartDashboard.putData("Static auto commands", m_autoChooser);

    // Publish to the dashboard any auto parameters that can be used to dynamically
    // create a composite auto command. These parameters are things like starting
    // position, actions, etc.
    m_dynamicAutoCreator.publishParameters();

    SmartDashboard.putData("Smoke Test", buildFullTestSequence());
  }

  private void setDefaultCommands() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    m_drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(
            () ->
                m_drive
                    .withVelocityX(getVelocityX())
                    .withVelocityY(getVelocityY())
                    .withRotationalRate(getRotationalRate())
                    .withRotationalDeadband(getRotationalDeadband())));

    //    m_shooter.getYawRotationalRate() * DriveConstants.MAX_ANGULAR_VELOCITY)));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled()
        .whileTrue(m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    // m_driverController.triangle().whileTrue(m_drivetrain.applyRequest(() -> brake));
    // m_driverController
    //     .circle()
    //     .whileTrue(
    //         m_drivetrain.applyRequest(
    //             () ->
    //                 point.withModuleDirection(
    //                     new Rotation2d(
    //                         -m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // Subsystem Defaults
    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.stopMotors(), m_shooter));
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.stopMotors(), m_intake));
    m_hopper.setDefaultCommand(new RunCommand(() -> m_hopper.stopMotors(), m_hopper));
  }

  /**
   * Use this method to map driver controls and commands please use <a href="
   * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&leftStick=Drive&aButton=Lock+wheels&xButton=Re-enable+manual+driving&yButton=Face+processor&leftBumper=Face+Left+Coral+Station&backButton=Reset+robot+orientation&rightBumper=Face+Right+Coral+Station&bButton=Face+reef+wall&leftTrigger=Slow+Mode&rightTrigger=Super+Slow+Mode&dpadLeft=&rightStick=Rotate&dpadDown=Climb&dpadUp=Raise+climb
   * ">this controller map</a> to update and view the current controls.
   */
  private void configureDriverControllers() {
    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    // m_driverController.povUp().whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
    // m_driverController.povDown().whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
    // m_driverController.povRight().whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
    // m_driverController.povLeft().whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Quarter Speed
    m_driverController.R2().onTrue(Commands.runOnce(() -> m_robotSpeed = 0.25));

    m_driverController.R2().onFalse(Commands.runOnce(() -> m_robotSpeed = 1.0));

    m_driverController.cross().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_driverController.circle().whileTrue(m_aimingCommands.alignWithTower());

    // Reset the field-centric heading on left bumper press.
    m_driverController.options().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    m_driverController
        .square()
        .whileTrue(new RunCommand(() -> System.out.println(DriveConstants.MAX_ANGULAR_VELOCITY)));
    // Shoot Ball
    m_driverController
        .R1()
        .whileTrue(
            new SequentialCommandGroup(
                Commands.run(() -> m_shooter.rampUpShooter(), m_shooter).withTimeout(2),
                new ParallelCommandGroup(Commands.run(() -> m_shooter.shootBall(), m_shooter))));
    // Set rumble on the driver conroller when the robot is shooting the balls
    m_driverController
        .R1()
        .and(new Trigger(() -> !HubStatus.isHubActive(3, 3)))
        .whileTrue(
            Commands.runEnd(
                () -> m_driverController.setRumble(RumbleType.kBothRumble, 1.0),
                () -> m_driverController.setRumble(RumbleType.kBothRumble, 0.0)));
    // Commands.run(() -> m_hopper.runHopper(HopperPowers.SHOOT), m_hopper))));

    // Align with hub
    m_driverController
        .triangle()
        .toggleOnTrue(
            m_drivetrain
                .applyRequest(
                    () ->
                        m_drive
                            .withVelocityX(getVelocityX())
                            .withVelocityY(getVelocityY())
                            .withRotationalRate(
                                m_shooter.getYawRotationalRate()
                                    * DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY))
                .until(() -> Math.abs(m_driverController.getRightX()) > 0.1));
  }

  /**
   * Use this method to map operator controller controls and commands please use <a href="
   * https://www.padcrafter.com/?templates=Driver+Controller&plat=1&rightTrigger=Quarter+Speed&leftStick=Drive&rightStick=Rotate&aButton=Align+with+Hub&bButton=Align+with+Tower
   * ">this controller map</a> to update and view the current controls.
   */
  private void configureOperatorControllers() {
    // m_operatorController
    //    .x()
    //    .whileTrue(Commands.runOnce(() -> m_shooter.runLeftMotorTest(500), m_shooter));

    // m_operatorController.y().onTrue(Commands.run(() -> m_shooter.runRightMotorTest(10),
    // m_shooter));
    // m_operatorController.a().onTrue(Commands.run(() -> m_shooter.runMotorsTest(10), m_shooter));
    // m_operatorController.b().onTrue(Commands.runOnce(() -> m_shooter.stopMotors(), m_shooter));

    // m_operatorController
    //    .y()
    //    .toggleOnTrue(Commands.run(() -> m_shooter.runLeftMotor(.9, 0), m_shooter));

    // **Shooter Commands**

    // **Intake Commands**

    // Deploy and Stow Intake
    // m_operatorController
    //     .leftBumper()
    //     .toggleOnTrue(
    //         new RunCommand(() -> m_intake.deployedPosition(), m_intake)
    //             .andThen(() -> m_intake.stowedPosition()));

    // m_operatorController
    //     .leftBumper()
    //     .and(() -> (m_intake.isDeployed()))
    //     .onTrue(Commands.runOnce(() -> m_intake.stowedPosition(), m_intake));
    // m_operatorController
    //     .leftBumper()
    //     .and(() -> (!m_intake.isDeployed()))
    //     .onTrue(Commands.runOnce(() -> m_intake.deployedPosition(), m_intake));

    // m_operatorController.leftBumper().onTrue(Commands.run(() -> m_intake.runMotors(0.8),
    // m_intake));
    // m_operatorController.b().onTrue(Commands.run(() -> m_intake.runMotors(0.8), m_intake));

    m_operatorController
        .x()
        .toggleOnTrue(
            new RunCommand(() -> m_shooter.rampUpShooter(m_testShooterRPM, true), m_shooter));

    m_operatorController
        .y()
        .whileTrue(
            new ParallelCommandGroup(
                Commands.run(() -> m_shooter.shootBall(m_testShooterRPM), m_shooter),
                Commands.run(() -> m_hopper.runHopper(HopperPowers.SHOOT), m_hopper)));
    // Spin Intake
    m_operatorController
        .leftTrigger()
        .toggleOnTrue(
            new ParallelCommandGroup(
                Commands.run(() -> m_intake.runSpinner(0.7), m_intake),
                Commands.run(() -> m_hopper.runHopper(HopperPowers.INTAKE))));

    // Spin Intake Reverse
    m_operatorController
        .leftBumper()
        .toggleOnTrue(
            new ParallelCommandGroup(
                Commands.run(() -> m_intake.runSpinner(-0.9), m_intake),
                Commands.run(() -> m_hopper.runHopper(HopperPowers.INTAKE_REVERSE))));

    // Lock on to the Hub
    m_operatorController
        .povUp()
        .toggleOnTrue(
            new ParallelCommandGroup(
                m_drivetrain.applyRequest(
                    () ->
                        m_drive
                            .withVelocityX(
                                -m_driverController.getLeftY() * DriveConstants.MAX_LINEAR_SPEED)
                            .withVelocityY(
                                -m_driverController.getLeftX() * DriveConstants.MAX_LINEAR_SPEED)
                            .withRotationalRate(
                                Aiming.getYawTxAdjustment(
                                    LimelightHelpers.getTX(VisionConstants.FRONT_LIMELIGHT_NAME)))
                            .withRotationalDeadband(getRotationalDeadband()))));

    // Change Shooter Trim
    m_operatorController.povLeft().onTrue(new InstantCommand(() -> changeTestRpm(-50)));

    m_operatorController.povRight().onTrue(new InstantCommand(() -> changeTestRpm(50)));
  }

  public Command getAutonomousCommand() {
    // Simple drive forward auton
    // final var idle = new SwerveRequest.Idle();
    // return Commands.sequence(
    //     // Reset our field centric heading to match the robot
    //     // facing away from our alliance station wall (0 deg).
    //     m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric(Rotation2d.kZero)),
    //     // Then slowly drive forward (away from us) for 5 seconds.
    //     m_drivetrain
    //         .applyRequest(() ->
    // m_drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
    //         .withTimeout(5.0),
    //     // Finally idle for the rest of auton
    //     m_drivetrain.applyRequest(() -> idle));

    // First see if a dynamic auto command has been defined
    Command auto_command = m_dynamicAutoCreator.getCommand();
    if (auto_command == null) {

      // If not, get the static auto command selected in the AutoChooser drop-down in the dashboard
      auto_command = m_autoChooser.getSelected();
    }

    return auto_command;
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The linear deadband in meters per second.
   */
  public double getDeadband() {
    return DriveConstants.MAX_LINEAR_SPEED * 0.1 * m_robotSpeed;
  }

  /**
   * Deadbands are a percentage of the joystick input. 0.1 means you don't want to move until the
   * joystick is pushed at least 10% in any direction (to prevent drift)
   *
   * @return The rotational deadband in radians per second.
   */
  public double getRotationalDeadband() {
    return DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY * 0.15 * m_robotSpeed;
  }

  public double getVelocityX() {
    return m_driverController.getLeftY() * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
  }

  public double getVelocityY() {
    return m_driverController.getLeftX() * DriveConstants.MAX_LINEAR_SPEED * m_robotSpeed;
  }

  public double getRotationalRate() {
    return -m_driverController.getRightX()
        * DriveConstants.MAX_TELEOP_ANGULAR_VELOCITY
        * m_robotSpeed;
  }

  public void resetFieldPosition(Pose2d position) {
    m_drivetrain.resetPose(position);
  }

  public void resetRobotRotation(Rotation2d rotation) {
    m_drivetrain.resetRotation(rotation);
  }

  public void changeTestRpm(double val) {
    m_testShooterRPM += val;
  }

  public Command buildFullTestSequence() {
    List<Command> steps = new ArrayList<>();

    // Reset all indicators first
    steps.add(
        Commands.runOnce(
            () -> {
              for (Testable t : testableSubsystems) {
                t.resetTestIndicators();
              }
            }));

    for (Testable t : testableSubsystems) {
      steps.add(t.test());
      steps.add(new WaitCommand(1.0));
    }

    return new SequentialCommandGroup(steps.toArray(new Command[0]));
  }
}
