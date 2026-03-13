package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;

public class Aiming {
  private static PIDController yawPID = new PIDController(0.15, 0, 0);

  /**
   * @param lensHeight Distance from the floor to the Limelight's lens in inches
   * @param goalHeight The height of the goal in inches
   * @param mountAngle The mounting angle of the limelight in degrees
   * @param offSetAngle The additional angle to the target in degrees
   * @return The horizontal distance from the Limelight to the apriltag
   */
  public static double calculateTagDistance(
      double lensHeight, double goalHeight, double mountAngle, double offSetAngle) {
    double angleToGoalRadians = Math.toRadians(mountAngle + offSetAngle);
    return (goalHeight - lensHeight) / Math.tan(angleToGoalRadians);
  }

  /**
   * @param tx The current tx given by the limelight
   * @return The turning power for the steering motors
   */
  public static double getYawTxAdjustment(double tx) {
    double value = yawPID.calculate(tx, 0);
    return value;
  }
}
