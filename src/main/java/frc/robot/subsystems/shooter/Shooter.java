package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.Optional;

public class Shooter extends SubsystemBase {
  private final SparkFlex m_shooterLeftMotor =
      new SparkFlex(ShooterConstants.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_shooterRightMotor =
      new SparkFlex(ShooterConstants.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_kickerMotor =
      new SparkFlex(ShooterConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder m_leftShooterMotorEncoder = m_shooterLeftMotor.getEncoder();
  private final RelativeEncoder m_rightShooterMotorEncoder = m_shooterRightMotor.getEncoder();
  private final RelativeEncoder m_kickerMotorEncoder = m_kickerMotor.getEncoder();

  private final SparkClosedLoopController m_leftShooterMotorController =
      m_shooterLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_rightMotorController =
      m_shooterRightMotor.getClosedLoopController();
  private final SparkClosedLoopController m_kickerMotorController =
      m_kickerMotor.getClosedLoopController();

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Shooter");
  private final NetworkTable m_limelight = m_inst.getTable("limelight");

  private final DoublePublisher m_leftShooterSpeedPub =
      m_table.getDoubleTopic("Left Shooter Motor Speed ").publish();
  private final DoublePublisher m_RightShooterSpeedPub =
      m_table.getDoubleTopic("Right Shooter Motor Speed ").publish();
  private final DoublePublisher m_kickerSpeedPub =
      m_table.getDoubleTopic("Kicker Motor Speed ").publish();
  private final DoublePublisher m_shooterTargetSpeedPub =
      m_table.getDoubleTopic("Shooter Motor Target Speed ").publish();
  private final DoublePublisher m_distanceRobotToTagPub =
      m_table.getDoubleTopic("Distance From Robot to AprilTag ").publish();

  private double distanceFromLimelightToAprilTag = 0;

  public Shooter() {

    // Left Motor Configurations
    SparkFlexConfig shooterLeftMotorConfig = new SparkFlexConfig();
    shooterLeftMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.K_P_ShOOTER);
    m_shooterLeftMotor.configure(
        shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right Motor Configurations
    SparkFlexConfig shooterRightMotorConfig = new SparkFlexConfig();
    shooterRightMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.K_P_ShOOTER);
    m_shooterRightMotor.configure(
        shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Kicker Motor Configurations
    SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
    kickerMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kBrake)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.K_P_KICKER);
    m_kickerMotor.configure(
        kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double leftShooterVelocity = m_leftShooterMotorEncoder.getVelocity();
    m_leftShooterSpeedPub.set(leftShooterVelocity);
    double rightShooterVelocity = m_rightShooterMotorEncoder.getVelocity();
    m_RightShooterSpeedPub.set(rightShooterVelocity);
    double kickerVelocity = m_kickerMotorEncoder.getVelocity();
    m_kickerSpeedPub.set(kickerVelocity);

    NetworkTableEntry ty = m_limelight.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // distance from the target to the floor
    double goalHeightInches = 60.0;

    double angleToGoalDegrees = VisionConstants.LIMELIGHT_MOUNT_ANGLE + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

    // calculate distance
    distanceFromLimelightToAprilTag =
        (goalHeightInches - VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE)
            / Math.tan(angleToGoalRadians);
    m_distanceRobotToTagPub.set(distanceFromLimelightToAprilTag);
  }

  public void runMotor(double power) {
    m_shooterLeftMotor.set(power);
    m_kickerMotor.set(power / 2);
  }

  public void runMotorTest(double setpoint) {
    m_leftShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
    m_shooterTargetSpeedPub.set(setpoint);
  }

  public void stopMotor() {
    m_shooterLeftMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public void rampUpShooter() {
    double tagID =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;
    } else if (alliance.get() == Alliance.Blue) {
      if (isHubActive() && (tagID == 26 || tagID == 25)) {
        m_leftShooterMotorController.setSetpoint(100, ControlType.kVelocity);
        m_kickerMotorController.setSetpoint(100, ControlType.kVelocity);
      } else {
        m_shooterLeftMotor.stopMotor();
        m_shooterRightMotor.stopMotor();
        m_kickerMotor.stopMotor();
      }
    } else if (alliance.get() == Alliance.Red) {
      if (isHubActive() && (tagID == 9 || tagID == 10)) {
        m_leftShooterMotorController.setSetpoint(100, ControlType.kVelocity);
        m_kickerMotorController.setSetpoint(100, ControlType.kVelocity);
      } else {
        m_shooterLeftMotor.stopMotor();
        m_shooterRightMotor.stopMotor();
        m_kickerMotor.stopMotor();
      }
    }
  }

  /** This method is used toi determine if the Hub is active */
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // If there is no game data, assume hub is inactive
    if (gameData.isEmpty()) {
      return false;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // Assum hub is inactive if we have invalid game data
        return false;
      }
    }

    // Shift is active for blue if red won auto, or red if blue won auto
    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      return false;
    }
  }
}
