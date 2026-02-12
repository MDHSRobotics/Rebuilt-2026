package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public final class Constants {
  private Constants() {}

  /** The update period of the robot in seconds. */
  public static final double UPDATE_PERIOD = 0.02;

  /** Robot Max Linear Velocity */
  public static final double MAX_LINEAR_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

  /** Robot Max Angular Velocity */
  public static final double MAX_ANGULAR_VELOCITY =
      RotationsPerSecond.of(.75).in(RadiansPerSecond); // 3/4 of a rotation per second

  public static class ControllerConstants {
    private ControllerConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
}
