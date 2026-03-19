package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeConstants extends SubsystemBase {
  private IntakeConstants() {}

  public static final int INTAKE_RIGHT_MOTOR_ID = 16;
  public static final int INTAKE_LEFT_MOTOR_ID = 17;
  public static final int INTAKE_SPINNERS_MOTOR_ID = 18;

  /**
   * The current limit for the intake and spinners in amps. This is currently set to the value
   * suggested by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;

  /** The proportional gain for the Intake */
  public static final double K_P = 0.5;

  /**
   * The position that the left motor intake can pick up the ball in rotations. Left and right is
   * determined by looking at the front of the robot
   */
  public static final double PICKUP_POSITION_LEFT = 0.305;

  /**
   * The position that the right motor intake can pick up the ball in rotations. Left and right is
   * determined by looking at the front of the robot
   */
  public static final double PICKUP_POSITION_RIGHT = 0.905;

  /**
   * The stowed position of the left motor intake in rotations Left and right is determined by
   * looking at the front of the robot
   */
  public static final double STOWED_POSITION_LEFT = 0.75;

  /**
   * The stowed position of the right motor intake in rotations Left and right is determined by
   * looking at the front of the robot
   */
  public static final double STOWED_POSITION_RIGHT = 0.440;

  /** The intake speed of the spinners in RPM */
  public static final double INTAKE_SPINNERS_SPEED = 0;
}
