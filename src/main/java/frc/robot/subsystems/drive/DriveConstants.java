package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;

/**
 * Contains constants for the swerve drive that haven't been specified in {@link
 * frc.robot.subsystems.drive.TunerConstants TunerConstants}.
 *
 * <p>Some constants are pre-converted to avoid repeated unit conversions in the main robot loop.
 */
public class DriveConstants {
  private DriveConstants() {}

  /** Distance between center to front bumper */
  public static final Distance CENTER_TO_FRONT_BUMPER_LENGTH = Inches.of(0);

  public static final Distance BUMPER_TO_BUMPER_X_DISTANCE = Inches.of(0);
  public static final Distance BUMPER_TO_BUMPER_Y_DISTANCE = Inches.of(0);

  /** Distance between front left module (cancoder) and front right module (cancoder) */
  private static final Distance FRONT_LEFT_AND_FRONT_RIGHT_CANCODER_DISTANCE =
      TunerConstants.kFrontLeftYPos.minus(TunerConstants.kFrontRightYPos);

  /** Distance between front left module (cancoder) and back left module (cancoder) */
  private static final Distance FRONT_LEFT_AND_BACK_LEFT_CANCODER_DISTANCE =
      TunerConstants.kFrontLeftXPos.minus(TunerConstants.kBackLeftXPos);

  /** Distance from center of robot to a module (cancoder) */
  public static final Distance CENTER_OF_ROBOT_TO_CANCODER_DISTANCE =
      Inches.of(
          Math.hypot(
              FRONT_LEFT_AND_FRONT_RIGHT_CANCODER_DISTANCE.div(2.0).in(Inches),
              FRONT_LEFT_AND_BACK_LEFT_CANCODER_DISTANCE.div(2.0).in(Inches)));
}
