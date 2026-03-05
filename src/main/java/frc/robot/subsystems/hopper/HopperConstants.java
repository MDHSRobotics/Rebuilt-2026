package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperConstants extends SubsystemBase {

  private HopperConstants() {}

  public static final int HOPPER_MOTOR_ID = 22;

  /**
   * The current limit for the hopper in amps. This is currently set to the value suggested by <a
   * href="https://docs.revrobotics.com/brushless/spark-flex/gs/make-it-spin#suggested-current-limits">REV
   * for the NEO Vortex.</a>
   */
  public static final int CURRENT_LIMIT = 80;

  /** The proportional gain for the Intake */
  public static final double K_P = 0;
}
