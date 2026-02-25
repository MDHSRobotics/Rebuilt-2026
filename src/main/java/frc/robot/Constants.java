package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

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

    // These needs to be tested and adjusted later on
    public static final double FRONT_X_STD_DEV = 0;
    public static final double FRONT_Y_STD_DEV = 0;
    public static Vector<N3> FRONT_STD_DEVS =
        VecBuilder.fill(FRONT_X_STD_DEV, FRONT_Y_STD_DEV, Double.MAX_VALUE);
  }

  public static class FieldConstants {
    private FieldConstants() {}

    public static final Translation3d[] NO_VISIBLE_TAGS = new Translation3d[0];
    public static final double[] NO_TAG_DISTANCES = new double[0];
  }
}
