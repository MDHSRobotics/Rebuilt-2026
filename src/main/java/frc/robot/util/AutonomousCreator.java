package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Consumer;

/** This util class is specific to our Pathplanner path naming scheme for Rebuilt-2026 */
public class AutonomousCreator {
  private final SendableChooser<String> m_startingPositionChooser = new SendableChooser<>();

  private final Consumer<Pose2d> m_odometryResetter;
  private final AutoTimer m_autoTimer = new AutoTimer();
  private Command m_autoSequence = null;

  public AutonomousCreator(Consumer<Pose2d> odometryResetter) {
    m_odometryResetter = odometryResetter;
  }

  public void sendAutoChooser() {
    // Starting position
  }
}
