package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
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

    // AprilTags poses to use for field positions
    public static final AprilTagFieldLayout APRILTAGS =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /**
     * Rotations of the Apriltags for aligning perpendicular to them.
     *
     * <p>These rotations correspond to the Z-rotation of the tags plus 180 degrees
     *
     * <p>Index into the array with the id number starting from 1.
     */
    public static final Rotation2d[] APRILTAG_ROTATIONS = {
      Rotation2d.kZero,
      APRILTAGS.getTagPose(1).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(2).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(3).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(4).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(5).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(6).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(7).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(8).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(9).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(10).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(11).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(12).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(13).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(14).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(15).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(16).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(17).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(18).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(19).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(20).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(21).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(22).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(23).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(24).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(25).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(26).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(27).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(28).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(29).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(30).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(31).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
      APRILTAGS.getTagPose(32).orElseThrow().getRotation().toRotation2d().plus(Rotation2d.k180deg),
    };
  }
}
