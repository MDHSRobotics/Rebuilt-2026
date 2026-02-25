package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.requests.DriveFacingAngle;
import java.util.function.DoubleSupplier;

/** This class provides instanced command factories for swerve drive aiming */
public class AimingCommand {
  private final CommandSwerveDrivetrain m_drivetrain;

  private final DoubleSupplier m_velocityXSupplier;
  private final DoubleSupplier m_velocityYSupplier;
  private final DoubleSupplier m_deadbandSupplier;

  // Joystick suppliers
  private final DoubleSupplier m_leftYSupplier;
  private final DoubleSupplier m_leftXSupplier;
  private final DoubleSupplier m_rightYSupplier;
  private final DoubleSupplier m_rightXSupplier;

  // NetworkTables
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_cameraTable = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

  // Swerve Request
  private final DriveFacingAngle m_driveFacingAngle =
      new DriveFacingAngle(
              DriveConstants.ROTATION_PID.kP,
              DriveConstants.MAX_ANGULAR_VELOCITY,
              DriveConstants.SWERVE_SETPOINT_GENERATOR,
              Constants.UPDATE_PERIOD)
          .withTolerance(DriveConstants.HEADING_TOLERANCE)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);

  /**
   * The current target by whichever command is running. You don't need to worry about multiple
   * commands accessing this because only one command can run at a time.
   */
  private Pose2d m_currentTargetPose;

  /** Logs the target pose to NetworkTables. Use this whenever you calculate a new target pose. */
  private final StructPublisher<Pose2d> m_targetPosePub =
      m_inst.getTable("DriveState").getStructTopic("Target Pose", Pose2d.struct).publish();

  /**
   * Gets the ID of the primary in-view apriltag.
   *
   * @see <a
   *     href="https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data">limelight
   *     NetworkTables API</a>
   * @see {@link frc.robot.util.LimelightHelpers#getFiducialID(String) LimelightHelpers equivalent}
   */
  private final IntegerSubscriber m_apriltagIDSub =
      m_cameraTable.getIntegerTopic("tid").subscribe(0);

  /**
   * Constructs an object that provides <a
   * href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced
   * command factories</a> for swerve drive aiming.
   *
   * @param drivetrain The drivetrain to drive and aim with.
   * @param velocityXSupplier A method reference or lambda that returns X velocity.
   * @param velocityYSupplierA method reference or lambda that returns Y velocity.
   * @param deadbandSupplier A method reference or lambda that returns deadband.
   * @param leftYSupplier A method reference or lambda that returns a left joystick's Y value.
   * @param leftXSupplier A method reference or lambda that returns a left joystick's X value.
   * @param rightYSupplier A method reference or lambda that returns a right joystick's Y value.
   * @param rightXSupplier A method reference or lambda that returns a right joystick's X value.
   */
  public AimingCommand(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier deadbandSupplier,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier rightYSupplier,
      DoubleSupplier rightXSupplier) {
    m_drivetrain = drivetrain;
    m_velocityXSupplier = velocityXSupplier;
    m_velocityYSupplier = velocityYSupplier;
    m_deadbandSupplier = deadbandSupplier;
    m_leftYSupplier = leftYSupplier;
    m_leftXSupplier = leftXSupplier;
    m_rightYSupplier = rightYSupplier;
    m_rightXSupplier = rightXSupplier;
  }
}
