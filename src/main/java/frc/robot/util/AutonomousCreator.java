package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Consumer;

/** This util class is specific to our Pathplanner path naming scheme for Rebuilt-2026 */
public class AutonomousCreator {
  private final SendableChooser<String> m_startingPositionChooser = new SendableChooser<>();
  private final SendableChooser<String> m_actionOneChooser = new SendableChooser<>();
  private final SendableChooser<String> m_createAuto = new SendableChooser<>();

  private final Consumer<Pose2d> m_odometryResetter;
  private final AutoTimer m_autoTimer = new AutoTimer();
  private Command m_autoSequence = null;

  // Subsystems
  private final Shooter m_shooter;

  public AutonomousCreator(Consumer<Pose2d> odometryResetter, Shooter shooter) {
    m_odometryResetter = odometryResetter;
    m_shooter = shooter;
  }

  public void sendAutoChooser() {
    // Starting position
    m_startingPositionChooser.addOption("Top", "Top to ");
    m_startingPositionChooser.addOption("Middle", "Middle to ");
    m_startingPositionChooser.addOption("Bottom", "Bottom to ");
    SmartDashboard.putData("Starting Position", m_startingPositionChooser);

    // Options for first action
    m_actionOneChooser.addOption("Action 1", "Shoot ball");

    // Create Auto
    SmartDashboard.putData("Choose Action and Create Auto", m_actionOneChooser);
  }

  private void createOneShootingSequenceAuto() {
    try {
      String pathName = m_startingPositionChooser.getSelected();
      pathName += m_actionOneChooser.getSelected();
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      m_autoSequence =
          Commands.sequence(
              resetOdometryCommand(path.getStartingHolonomicPose().orElseThrow()),
              Commands.waitSeconds(1),
              Commands.deadline(
                  AutoBuilder.followPath(path),
                  Commands.runOnce(m_autoTimer::resetAndStart),
                  Commands.run(() -> m_shooter.rampUpShooter(2000, true)).withTimeout(1),
                  Commands.run(() -> m_shooter.shootBall(2000)),
                  Commands.runOnce(m_autoTimer::stopAndPublish)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
      System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());
      m_autoSequence = Commands.none();
      return;
    }
  }

  public Command resetOdometryCommand(Pose2d startingPose) {
    return Commands.runOnce(
        () -> {
          Pose2d newStartingPose = startingPose;
          if (DriverStation.getAlliance().orElseThrow() == Alliance.Red) {
            newStartingPose = FlippingUtil.flipFieldPose(startingPose);
          }
          m_odometryResetter.accept(newStartingPose);
        });
  }

  public Command getAutonomousCommand() {
    return m_autoSequence;
  }
}
