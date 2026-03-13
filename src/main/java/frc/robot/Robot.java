// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.urcl.URCL;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private DoublePublisher m_matchTimePub;
  private boolean m_hasAppliedRobotRotation;

  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Silence Joystick warnings because they get in the way of other warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    m_matchTimePub = NetworkTableInstance.getDefault().getDoubleTopic("Match Time").publish();

    // Configure Limelight Positions
    LimelightHelpers.setCameraPose_RobotSpace(
        VisionConstants.FRONT_LIMELIGHT_NAME,
        VisionConstants.FRONT_LIMELIGHT_FORWARD_DISTANCE,
        0,
        VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE,
        0,
        VisionConstants.FRONT_LIMELIGHT_PITCH,
        0);
    // LimelightHelpers.setCameraPose_RobotSpace(
    //     VisionConstants.BACK_LIMELIGHT_NAME,
    //     VisionConstants.BACK_LIMELIGHT_FORWARD_DISTANCE,
    //     0,
    //     VisionConstants.BACK_LIMELIGHT_UP_DISTANCE,
    //     0,
    //     0,
    //     VisionConstants.BACK_LIMELIGHT_YAW);
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 1);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 1);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 200);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 200);

    SignalLogger.setPath("/media/sda1/logs/");
    SignalLogger.start();
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    URCL.start();
    FollowPathCommand.warmupCommand().schedule();
    m_hasAppliedRobotRotation = false;
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
    m_matchTimePub.set(DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    if (!m_hasAppliedRobotRotation) {
      Alliance alliance = DriverStation.getAlliance().orElse(null);
      if (alliance == Alliance.Blue) {
        m_robotContainer.resetRobotRotation(Rotation2d.k180deg);
        m_hasAppliedRobotRotation = true;
      } else if (alliance == Alliance.Red) {
        m_robotContainer.resetRobotRotation(Rotation2d.kZero);
        m_hasAppliedRobotRotation = true;
      }
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      System.out.println("Starting Auto: " + m_autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 2);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 2);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 2);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 4);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 0);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 0);
    m_robotContainer.resetFieldPosition(
        new Pose2d(Meters.of(10), Meters.of(5), Rotation2d.fromDegrees(180)));
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    SignalLogger.stop();
    LimelightHelpers.SetIMUMode(VisionConstants.FRONT_LIMELIGHT_NAME, 1);
    // LimelightHelpers.SetIMUMode(VisionConstants.BACK_LIMELIGHT_NAME, 1);
    LimelightHelpers.SetThrottle(VisionConstants.FRONT_LIMELIGHT_NAME, 200);
    // LimelightHelpers.SetThrottle(VisionConstants.BACK_LIMELIGHT_NAME, 200);
  }

  @Override
  public void simulationPeriodic() {}
}
