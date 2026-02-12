package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;

/**
 * IMPORTANT NOTE: Even while in simulation, the pose reported by this telemetry still represents a
 * robot pose that <a
 * href="https://shenzhen-robotics-alliance.github.io/maple-sim/simulation-details/#odometry-vision-simulation">
 * accumulates odometry errors. </a>
 *
 * <p>If you want to see the robot's actual position in simulation, use the pose logged by {@link
 * frc.robot.subsystems.drive.MapleSimSwerveDrivetrain Maple-sim}.
 */
public class DriveTelemetry {
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();

  /* Robot swerve drive state */
  private final NetworkTable m_driveStateTable = m_inst.getTable("DriveState");
  private final StructPublisher<Pose2d> m_drivePosePub =
      m_driveStateTable.getSubTable("Poses").getStructTopic("Pose", Pose2d.struct).publish();
  private final StructPublisher<ChassisSpeeds> m_speedsPub =
      m_driveStateTable
          .getSubTable("Speeds")
          .getStructTopic("Actual Robot-relative Speeds", ChassisSpeeds.struct)
          .publish();
  // private final StructArrayPublisher<SwerveModuleState> m_moduleStatesPub = m_driveStateTable
  //         .getStructArrayTopic("ModuleStates", SwerveModuleState.struct)
  //         .publish();
  // private final StructArrayPublisher<SwerveModuleState> m_moduleTargetsPub = m_driveStateTable
  //         .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct)
  //         .publish();
  // private final StructArrayPublisher<SwerveModulePosition> m_modulePositionsPub =
  // m_driveStateTable
  //         .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct)
  //         .publish();
  private final DoublePublisher m_odometryFrequencyPub =
      m_driveStateTable.getDoubleTopic("OdometryFrequency").publish();
  // private final DoublePublisher m_odometryPeriodPub =
  //         m_driveStateTable.getDoubleTopic("OdometryPeriod").publish();
  private final DoublePublisher m_linearSpeedPub =
      m_driveStateTable.getDoubleTopic("Linear Speed").publish();

  /** Accept the swerve drive state and log it to NetworkTables and SignalLogger. */
  public void telemeterize(SwerveDriveState state) {
    long timestampMicroseconds = stateTimestampToNTTimestamp(state.Timestamp);

    /* Telemeterize the swerve drive state */
    m_drivePosePub.set(state.Pose, timestampMicroseconds);
    m_speedsPub.set(state.Speeds, timestampMicroseconds);
    // m_moduleStatesPub.set(state.ModuleStates, timestampMicroseconds);
    // m_moduleTargetsPub.set(state.ModuleTargets, timestampMicroseconds);
    // m_modulePositionsPub.set(state.ModulePositions, timestampMicroseconds);
    m_odometryFrequencyPub.set(1.0 / state.OdometryPeriod, timestampMicroseconds);
    // m_odometryPeriodPub.set(state.OdometryPeriod, timestampMicroseconds);

    double linearSpeed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
    m_linearSpeedPub.set(linearSpeed, timestampMicroseconds);
  }

  /**
   * Converts a {@link com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState SwerveDriveState}
   * timestamp to the timebase required for NetworkTables.
   *
   * <p>This is basically the inverse of calling {@link
   * com.ctre.phoenix6.Utils#fpgaToCurrentTime(double) Utils.fpgaToCurrentTime()}.
   *
   * @param stateTimestampSeconds The SwerveDriveState timestamp in seconds
   * @return The equivalent NetworkTables timestamp in microseconds
   */
  public static long stateTimestampToNTTimestamp(double stateTimestampSeconds) {
    double NTTimestampSeconds =
        stateTimestampSeconds + (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds());
    return (long) (NTTimestampSeconds * 1000000.0);
  }
}
