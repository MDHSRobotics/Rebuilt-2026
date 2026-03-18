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
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.LoggedTunableNumber;
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
  private final SparkClosedLoopController m_rightShooterMotorController =
      m_shooterRightMotor.getClosedLoopController();

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Shooter");
  private final NetworkTable m_limelight = m_inst.getTable("limelight");

  private final DoublePublisher m_leftCurrentVelocityPub =
      m_table.getDoubleTopic("Left Shooter Current Velocity ").publish();
  private final DoublePublisher m_rightCurrentVelocityPub =
      m_table.getDoubleTopic("Right Shooter Current Velocity ").publish();
  private final DoublePublisher m_leftTargetVelocityPub =
      m_table.getDoubleTopic("Left Shooter Target Velocity ").publish();
  private final DoublePublisher m_rightTargetVelocityPub =
      m_table.getDoubleTopic("Right Shooter Target Velocity ").publish();
  private final DoublePublisher m_kickerCurrentVelocityPub =
      m_table.getDoubleTopic("Kicker Motor Current Velocity ").publish();
  private final DoublePublisher m_distanceRobotToTagPub =
      m_table.getDoubleTopic("Distance From Robot to AprilTag ").publish();

  private double distanceFromLimelightToAprilTag = 0;
  private double shooter_trim = 0;

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_P_SHOOTER);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_I_SHOOTER);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_D_SHOOTER);

  SparkFlexConfig shooterLeftMotorConfig;

  public Shooter() {

    // Left Motor Configurations
    shooterLeftMotorConfig = new SparkFlexConfig();
    shooterLeftMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP.get(), kI.get(), kD.get());
    m_shooterLeftMotor.configure(
        shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Right Motor Configurations
    SparkFlexConfig shooterRightMotorConfig = new SparkFlexConfig();
    shooterRightMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            ShooterConstants.K_P_SHOOTER,
            ShooterConstants.K_I_SHOOTER,
            ShooterConstants.K_D_SHOOTER);
    m_shooterRightMotor.configure(
        shooterRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Kicker Motor Configurations
    SparkFlexConfig kickerMotorConfig = new SparkFlexConfig();
    kickerMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    m_kickerMotor.configure(
        kickerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    /* Logging */
    m_leftTargetVelocityPub.set(m_leftShooterMotorController.getSetpoint());
    m_leftCurrentVelocityPub.set(m_leftShooterMotorEncoder.getVelocity());
    m_rightTargetVelocityPub.set(m_rightShooterMotorController.getSetpoint());
    m_rightCurrentVelocityPub.set(m_rightShooterMotorEncoder.getVelocity());
    m_kickerCurrentVelocityPub.set(m_kickerMotorEncoder.getVelocity());

    NetworkTableEntry ty = m_limelight.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    double angleToGoalDegrees = VisionConstants.LIMELIGHT_MOUNT_ANGLE + targetOffsetAngle_Vertical;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

    // calculate distance
    distanceFromLimelightToAprilTag =
        (FieldConstants.DISTANCE_FROM_FLOOR_TO_HUB_TAG
                - VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE)
            / Math.tan(angleToGoalRadians);
    m_distanceRobotToTagPub.set(distanceFromLimelightToAprilTag);

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      shooterLeftMotorConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
    }
  }

  public void runLeftMotor(double power, double kickerPower) {
    m_shooterLeftMotor.set(power);
    m_kickerMotor.set(kickerPower);
  }

  public void runLeftMotorTest(double setpoint) {
    m_leftShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public void runRightMotorTest(double setpoint) {
    m_rightShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public void runMotorsTest(double setpoint) {
    m_rightShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
    m_leftShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
  }

  public void stopMotors() {
    m_shooterLeftMotor.stopMotor();
    m_shooterRightMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public void shootBall(double setpoint) {
    m_leftShooterMotorController.setSetpoint(setpoint, ControlType.kVelocity);
    m_kickerMotor.set(ShooterConstants.KICKER_SPEED);
  }

  public void changeTrim(double amount) {
    shooter_trim += amount;
  }

  public void rampUpShooter(double rpm, boolean test) {
    // Account for whether we are in a testing mode, or in an actual match or practice match
    if (test) {
      m_leftShooterMotorController.setSetpoint(rpm, ControlType.kVelocity);
      return;
    }
    double tagID =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
    double targetSpeed = rpm;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;
    }
    boolean hubActive = isHubActive() || test;
    if (alliance.get() == Alliance.Blue) {
      if (hubActive) { // && (tagID == 26 || tagID == 25)) {
        m_leftShooterMotorController.setSetpoint(targetSpeed, ControlType.kVelocity);
      } else {
        m_shooterLeftMotor.stopMotor();
        m_shooterRightMotor.stopMotor();
        m_kickerMotor.stopMotor();
      }
    } else if (alliance.get() == Alliance.Red) {
      if (hubActive) { // && (tagID == 9 || tagID == 10)) {
        m_leftShooterMotorController.setSetpoint(targetSpeed, ControlType.kVelocity);
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
      return true;
    }
  }
}
