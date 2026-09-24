package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants.HopperPowers;
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

  private final Consumer<Pose2d> m_odometryResetter;
  private final AutoTimer m_autoTimer = new AutoTimer();
  private Command m_dynamicAutoSequence = null;

  // Subsystems
  private final Shooter m_shooter;
  private final Hopper m_hopper;
  public final Drive m_drivetrain;

  public DynamicAutoCreator(
      Consumer<Pose2d> odometryResetter, Shooter shooter, Hopper hopper, Drive drivetrain) {
    m_odometryResetter = odometryResetter;
    m_shooter = shooter;
    m_hopper = hopper;
    m_drivetrain = drivetrain;
  }

  /*
   * This method publishes to the dashboard parameter settings that can be used to dynamically
   * create an auto command. These parameters are things like starting position and
   * shooting strategy.
   */
  public void publishParameters() {

    // Select whether to use a dynamic or static auto command
    m_autoType.addOption("Dynamic", "Dynamic");
    m_autoType.setDefaultOption("Static", "Static");
    m_autoType.onChange(this::updateDynamicCommand);
    SmartDashboard.putData("Type of Auto Command", m_autoType);

    // Starting position option
    m_startingPositionChooser.setDefaultOption("Top", "Top to ");
    m_startingPositionChooser.addOption("Middle", "Middle to ");
    m_startingPositionChooser.addOption("Bottom", "Bottom to ");
    m_startingPositionChooser.onChange(this::updateDynamicCommand);
    SmartDashboard.putData("Starting Position", m_startingPositionChooser);

    // Options for first action
    m_actionOneChooser.setDefaultOption("Shoot", "Shoot ball");
    m_actionOneChooser.onChange(this::updateDynamicCommand);
    SmartDashboard.putData("Action 1", m_actionOneChooser);

    // Try to generate a dynamic auto command based on the initial parameter settings
    updateDynamicCommand("");
  }

  private void updateDynamicCommand(String changedSetting) {
    String autoType = m_autoType.getSelected();
    if (autoType.equals("Static")) {
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

  public Command createShootingAutoSequence() {
    Command auto_command =
        new SequentialCommandGroup(
            Commands.run(() -> m_shooter.rampUpShooter(), m_shooter).withTimeout(4),
            new ParallelCommandGroup(
                Commands.run(() -> m_hopper.runHopper(HopperPowers.INTAKE), m_hopper),
                Commands.run(() -> m_shooter.shootBall(), m_shooter).withTimeout(6)));
    return auto_command;
  }

  public Command createMiddleShootingAutoSequence() {
    return new SequentialCommandGroup(
        m_drivetrain.runOnce(
            () ->
                m_drivetrain.setPose(
                    new Pose2d(m_drivetrain.getPose().getTranslation(), Rotation2d.kZero))),
        Commands.run(() -> runFieldRelative(-1.0, 0.0, 0.0), m_drivetrain).withTimeout(1.5),
        m_drivetrain.runOnce(m_drivetrain::stop),
        Commands.waitSeconds(0.5),
        createShootingAutoSequence());
  }

  public Command createMiddleShootingRampAutoSequence() {
    return new SequentialCommandGroup(
        m_drivetrain.runOnce(
            () ->
                m_drivetrain.setPose(
                    new Pose2d(m_drivetrain.getPose().getTranslation(), Rotation2d.kZero))),
        Commands.run(() -> runFieldRelative(-1.0, 0.0, 0.0), m_drivetrain).withTimeout(1.0),
        m_drivetrain.runOnce(m_drivetrain::stop),
        Commands.waitSeconds(0.5),
        Commands.run(() -> runFieldRelative(0.0, 1.0, 0.0), m_drivetrain).withTimeout(2.5),
        m_drivetrain.runOnce(m_drivetrain::stop));
  }

  private void runFieldRelative(double vx, double vy, double omega) {
    m_drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, m_drivetrain.getRotation()));
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
