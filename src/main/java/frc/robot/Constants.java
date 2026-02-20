package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {
  private Constants() {}

  /** The update period of the robot in seconds. */
  public static final double UPDATE_PERIOD = 0.02;

  public static class ControllerConstants {
    private ControllerConstants() {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class VisionConstants {
    private VisionConstants() {}

    public static final String FRONT_LIMELIGHT_NAME = "limelight";

    /** Distance from the center of the robot to the front limelight lens in meters */
    public static final double FRONT_LIMELIGHT_FORWARD_DISTANCE = Inches.of(0).in(Meters);

    /** Distance from the floor to the front limelight lens in meters */
    public static final double FRONT_LIMELIGHT_UP_DISTANCE = Inches.of(0).in(Meters);

    /** Units: Degrees */
    public static final double FRONT_LIMELIGHT_PITCH = 0;
  }
}
