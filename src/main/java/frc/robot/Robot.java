// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
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
        0,
        0);

    SignalLogger.setPath("/media/sda1/logs/");
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
    URCL.start();
    FollowPathCommand.warmupCommand().schedule();
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
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      System.out.println("Starting Auto: " + m_autonomousCommand.getName());
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
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
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    SignalLogger.stop();
  }

  @Override
  public void simulationPeriodic() {}
}
