package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeConstants extends SubsystemBase {
  private IntakeConstants() {}

  public static final int INTAKE_TOP_MOTOR_ID = 16;
  public static final int INTAKE_BOTTOM_MOTOR_ID = 17;
  public static final int INTAKE_SPINNERS_MOTOR_ID = 18;

  /**
   * The current limit for the arm and wheels in amps. This is currently set to the value suggested
   * by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;
}
