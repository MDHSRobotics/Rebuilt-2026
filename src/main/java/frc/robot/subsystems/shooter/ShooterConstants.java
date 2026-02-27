package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterConstants extends SubsystemBase {
  public static final int SHOOTER_LEFT_MOTOR_ID = 19;
  public static final int SHOOTER_RIGHT_MOTOR_ID = 20;
  public static final int KICKER_MOTOR_ID = 21;

  /**
   * The current limit for the arm and wheels in amps. This is currently set to the value suggested
   * by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;
}
