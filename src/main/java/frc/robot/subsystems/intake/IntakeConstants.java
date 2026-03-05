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
  public static final double K_P = 0;

  /** The position that the intake can pick up the ball in rotations */
  public static final double PICKUP_POSITION = 0;

  /** The stowed position of the intake in rotations */
  public static final double STOWED_POSITION = 0;

  /** The intake speed of the spinners in RPM */
  public static final double INTAKE_SPINNERS_SPEED = 0;
}
