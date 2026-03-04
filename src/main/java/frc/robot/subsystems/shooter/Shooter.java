package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

  private final DoublePublisher m_leftShooterSpeedPub =
      m_table.getDoubleTopic("Left Shooter Motor Speed ").publish();
  private final DoublePublisher m_RightShooterSpeedPub =
      m_table.getDoubleTopic("Right Shooter Motor Speed ").publish();
  private final DoublePublisher m_kickerSpeedPub =
      m_table.getDoubleTopic("Kicker Motor Speed ").publish();

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
  }

  public void runMotor(double power) {
    m_shooterLeftMotor.set(power);
    m_kickerMotor.set(power / 2);
  }

  public void stopMotor() {
    m_shooterLeftMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }
}
