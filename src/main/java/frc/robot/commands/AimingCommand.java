package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.function.DoubleSupplier;

/** This class provides instanced command factories for swerve drive aiming */
public class AimingCommand {
  private final Drive m_drivetrain;

  private final DoubleSupplier m_velocityXSupplier;
  private final DoubleSupplier m_velocityYSupplier;
  private final DoubleSupplier m_deadbandSupplier;

  // NetworkTables
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_cameraTable = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

  // Swerve Request
  private final ProfiledPIDController m_angleController =
      new ProfiledPIDController(
          DriveConstants.ROTATION_PID.kP,
          DriveConstants.ROTATION_PID.kI,
          DriveConstants.ROTATION_PID.kD,
          new TrapezoidProfile.Constraints(
              DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_ACCELERATION));

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

  private Rotation2d m_targetRotation = Rotation2d.kZero;

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
      Drive drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier deadbandSupplier) {

    m_drivetrain = drivetrain;
    m_velocityXSupplier = velocityXSupplier;
    m_velocityYSupplier = velocityYSupplier;
    m_deadbandSupplier = deadbandSupplier;

    m_angleController.enableContinuousInput(-Math.PI, Math.PI);
    m_angleController.setTolerance(DriveConstants.HEADING_TOLERANCE.in(Radians));
  }

  public Command alignWithTower() {
    return m_drivetrain.startRun(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

          if (alliance == Alliance.Blue) {
            m_targetRotation = FieldConstants.APRILTAG_ROTATIONS[31];
          } else {
            m_targetRotation = FieldConstants.APRILTAG_ROTATIONS[15];
          }

          m_angleController.reset(m_drivetrain.getRotation().getRadians());
        },
        this::driveFacingTarget);
  }

  public Command alignWithHub() {
    return m_drivetrain.startRun(
        () -> {
          Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

          if (alliance == Alliance.Blue) {
            m_targetRotation = FieldConstants.APRILTAG_ROTATIONS[25];
          } else {
            m_targetRotation = FieldConstants.APRILTAG_ROTATIONS[9];
          }

          m_angleController.reset(m_drivetrain.getRotation().getRadians());
        },
        this::driveFacingTarget);
  }

  private void driveFacingTarget() {
    double vx = m_velocityXSupplier.getAsDouble();
    double vy = m_velocityYSupplier.getAsDouble();

    // Reproduce the translational deadband from the old DriveFacingAngle request.
    Translation2d velocity = new Translation2d(vx, vy);
    if (velocity.getNorm() < m_deadbandSupplier.getAsDouble()) {
      velocity = Translation2d.kZero;
    }

    double omega =
        m_angleController.calculate(
            m_drivetrain.getRotation().getRadians(), m_targetRotation.getRadians());

    Rotation2d fieldRelativeRotation = m_drivetrain.getRotation();

    // Maintain the same driver perspective on both alliances.
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      fieldRelativeRotation = fieldRelativeRotation.plus(Rotation2d.fromDegrees(180.0));
    }

    m_drivetrain.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), omega, fieldRelativeRotation));
  }
}
