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

/**
 * This util class is specific to our Pathplanner path naming scheme for Rebuilt-2026. It can use
 * the individual paths defined in Pathplanner to dynamically create a composite auto command
 * consisting of paths and actions. This is done using parameters which can be set in the dashboard.
 */
public class DynamicAutoCreator {
  private final SendableChooser<String> m_autoType = new SendableChooser<>();
  private final SendableChooser<String> m_startingPositionChooser = new SendableChooser<>();
  private final SendableChooser<String> m_actionOneChooser = new SendableChooser<>();
  private final SendableChooser<String> m_middleSequenceChooser = new SendableChooser<>();
  private final SendableChooser<String> m_topSequenceChooser = new SendableChooser<>();
  private final SendableChooser<String> m_bottomSequenceChooser = new SendableChooser<>();

  private final Consumer<Pose2d> m_odometryResetter;
  private final AutoTimer m_autoTimer = new AutoTimer();
  private Command m_dynamicAutoSequence = null;

  // Subsystems
  private final Shooter m_shooter;

  public DynamicAutoCreator(Consumer<Pose2d> odometryResetter, Shooter shooter) {
    m_odometryResetter = odometryResetter;
    m_shooter = shooter;
  }

  /*
   * This method publishes to the dashboard parameter settings that can be used to dynamically
   * create an auto command. These parameters are things like starting position and
   * shooting strategy.
   */
  public void publishParameters() {

    // Select whether to use a dynamic or static auto command
    m_autoType.setDefaultOption("Dynamic", "Dynamic");
    m_autoType.addOption("Static", "Static");
    m_autoType.onChange(this::updateDynamicCommand);
    SmartDashboard.putData("Type of Auto Command", m_autoType);

    // Starting position option
    m_startingPositionChooser.setDefaultOption("Top", "Top to ");
    m_startingPositionChooser.addOption("Middle", "Middle to ");
    m_startingPositionChooser.addOption("Bottom", "Bottom to ");
    SmartDashboard.putData("Starting Position", m_startingPositionChooser);

    // Options for first action
    m_actionOneChooser.setDefaultOption("Shoot", "Shoot ball");
    SmartDashboard.putData("Action 1", m_actionOneChooser);

    // Options for full auto sequence
    m_middleSequenceChooser.addOption("Middle 2 Auto Sequences", "Middle 2 Auto Sequences");
    m_topSequenceChooser.addOption("Top 2 Auto Sequences", "Top 2 Auto Sequences");
    m_bottomSequenceChooser.addOption("Bottom 2 Auto Sequences", "Bottom 2 Auto Sequences");

    // Try to generate a dynamic auto command based on the initial parameter settings
    updateDynamicCommand("Dynamic");
  }

  private void updateDynamicCommand(String autoType) {
    if (autoType == "Static") {
      // Static chosen so clear dynamic command
      m_dynamicAutoSequence = null;
    } else {
      // Create a dynamic command based on current settings of auto parameters
      createOneShootingSequenceAuto();
    }
  }

  private void createOneShootingSequenceAuto() {
    try {
      String pathName = m_startingPositionChooser.getSelected();
      pathName += m_actionOneChooser.getSelected();
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      m_dynamicAutoSequence =
          Commands.sequence(
              resetOdometryCommand(path.getStartingHolonomicPose().orElseThrow()),
              Commands.waitSeconds(1),
              Commands.deadline(
                  AutoBuilder.followPath(path),
                  Commands.runOnce(m_autoTimer::resetAndStart),
                  Commands.run(() -> m_shooter.rampUpShooter()).withTimeout(0.5),
                  Commands.run(() -> m_shooter.shootBall()),
                  Commands.runOnce(m_autoTimer::stopAndPublish)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path: " + e.getMessage(), e.getStackTrace());
      System.out.println("Failed to schedule auto path" + e.getMessage() + e.getStackTrace());
      m_dynamicAutoSequence = null;
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

  /* This method returns the dynamicly-generated auto command based on
   * options set in the dashboard. If no settings have been selected, return null.
   */
  public Command getCommand() {
    return m_dynamicAutoSequence;
  }
}
