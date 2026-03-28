package frc.robot.subsystems.shooter;

import static frc.robot.util.EpsilonEquals.epsilonEquals;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Aiming;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PolynomialInterpolation;
import frc.robot.util.Testable;

public class Shooter extends SubsystemBase implements Testable {
  /* Spark Flex Motors */
  private final SparkFlex m_shooterLeftMotor =
      new SparkFlex(ShooterConstants.SHOOTER_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_shooterRightMotor =
      new SparkFlex(ShooterConstants.SHOOTER_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_kickerMotor =
      new SparkFlex(ShooterConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

  /* Motor Encoders */
  private final RelativeEncoder m_leftShooterMotorEncoder = m_shooterLeftMotor.getEncoder();
  private final RelativeEncoder m_rightShooterMotorEncoder = m_shooterRightMotor.getEncoder();
  private final RelativeEncoder m_kickerMotorEncoder = m_kickerMotor.getEncoder();

  /* Shooter Motors Controllers */
  private final SparkClosedLoopController m_leftShooterMotorController =
      m_shooterLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_rightShooterMotorController =
      m_shooterRightMotor.getClosedLoopController();

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Shooter");
  private final NetworkTable m_limelight = m_inst.getTable("limelight");

  /* Networktables Publishers */
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
  private final DoublePublisher m_txAdjustmentPub =
      m_table.getDoubleTopic("TX Adjustment").publish();

  private final NetworkTableEntry m_kickerMotorOk =
      m_inst.getTable("Test").getEntry("IntakeSpinnerMotorRPM_OK");
  private final NetworkTableEntry m_leftMotorOk =
      m_inst.getTable("Test").getEntry("ShooterLeftMotorRPM_OK");
  private final NetworkTableEntry m_rightMotorOk =
      m_inst.getTable("Test").getEntry("ShooterRightMotorRPM_OK");

  private NetworkTableEntry ty = m_limelight.getEntry("ty");
  private NetworkTableEntry tx = m_limelight.getEntry("tx");

  private NetworkTableEntry tid = m_limelight.getEntry("tid");

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Shooter/kP", ShooterConstants.K_P_SHOOTER);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Shooter/kI", ShooterConstants.K_I_SHOOTER);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Shooter/kD", ShooterConstants.K_D_SHOOTER);

  private double m_leftTargetVelocity = 0;
  private double m_rightTargetVelocity = 0;
  private double m_currentDistance = 0;
  private double m_lastDistance = 0;

  private double m_shooterTrim = 50;
  private boolean m_tagIsSeen = false;

  SparkFlexConfig shooterLeftMotorConfig;

  private PolynomialInterpolation polynomial =
      new PolynomialInterpolation(ShooterConstants.DISTANCES, ShooterConstants.RPMS);

  public Shooter() {

    // Left Motor Configurations
    shooterLeftMotorConfig = new SparkFlexConfig();
    shooterLeftMotorConfig
        .smartCurrentLimit(ShooterConstants.CURRENT_LIMIT)
        .idleMode(IdleMode.kCoast)
        .inverted(false)
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
        .pid(kP.get(), kI.get(), kD.get());
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

    SmartDashboard.putBoolean("AprilTag is recognized", m_tagIsSeen);
  }

  @Override
  public void periodic() {
    /* Logging */
    m_leftTargetVelocityPub.set(m_leftTargetVelocity + m_shooterTrim);
    m_leftCurrentVelocityPub.set(m_leftShooterMotorEncoder.getVelocity());
    m_rightTargetVelocityPub.set(m_rightTargetVelocity);
    m_rightCurrentVelocityPub.set(m_rightShooterMotorEncoder.getVelocity());
    m_kickerCurrentVelocityPub.set(m_kickerMotorEncoder.getVelocity());

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      shooterLeftMotorConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
      m_shooterLeftMotor.configure(
          shooterLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    m_currentDistance =
        Aiming.calculateTagDistance(
            VisionConstants.FRONT_LIMELIGHT_UP_DISTANCE_INCHES,
            FieldConstants.DISTANCE_FROM_FLOOR_TO_HUB_TAG,
            VisionConstants.LIMELIGHT_MOUNT_ANGLE,
            ty.getDouble(0));
    if (tid.getDouble(0) > 0) {
      m_tagIsSeen = true;
      m_lastDistance = m_currentDistance;
    } else {
      m_tagIsSeen = false;
      m_currentDistance = m_lastDistance;
    }

    m_distanceRobotToTagPub.set(m_currentDistance);
  }

  public void runLeftMotor(double power, double kickerPower) {
    m_shooterLeftMotor.set(power);
    m_kickerMotor.set(kickerPower);
  }

  public void stopMotors() {
    m_shooterLeftMotor.stopMotor();
    m_shooterRightMotor.stopMotor();
    m_kickerMotor.stopMotor();
  }

  public void shootBall() {
    rampUpShooter();
    m_kickerMotor.set(ShooterConstants.KICKER_SPEED);
  }

  public void shootBall(double rpm) {
    rampUpShooter(rpm);
    m_kickerMotor.set(ShooterConstants.KICKER_SPEED);
  }

  public void changeTrim(double amount) {
    m_shooterTrim += amount;
  }

  private void setLeftSetpoint(double rpm) {
    m_leftTargetVelocity = rpm;
    m_leftShooterMotorController.setSetpoint(rpm, ControlType.kVelocity);
  }

  private void setRightSetpoint(double rpm) {
    m_rightTargetVelocity = rpm;
    m_rightShooterMotorController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void rampUpShooter() {
    double targetRPM = Aiming.calculateShooterRPM(polynomial, m_currentDistance, tid.getDouble(0));
    targetRPM += m_shooterTrim;
    setLeftSetpoint(targetRPM);
  }

  public void rampUpShooter(double rpm) {
    double targetRPM = rpm;
    targetRPM += m_shooterTrim;
    setLeftSetpoint(targetRPM);
  }

  public double getYawRotationalRate() {
    double txAdjusment = Aiming.getYawTxAdjustment(tx.getDouble(0));
    m_txAdjustmentPub.set(txAdjusment);
    return txAdjusment;
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_kickerMotor.set(ShooterConstants.TEST_POWER);
                  double rpm = m_kickerMotorEncoder.getVelocity();
                  m_kickerMotorOk.setBoolean(rpm > ShooterConstants.TEST_RPM);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_kickerMotor.stopMotor();
                  ;
                  m_kickerMotorOk.setBoolean(Math.abs(m_kickerMotorEncoder.getVelocity()) < 5);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        new WaitCommand(ShooterConstants.TEST_TIMEOUT / 4),
        Commands.run(
                () -> {
                  setLeftSetpoint(ShooterConstants.TEST_RPM_2);
                  m_leftMotorOk.setBoolean(
                      epsilonEquals(
                          m_leftShooterMotorEncoder.getVelocity(),
                          ShooterConstants.TEST_RPM_2,
                          200));
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  setLeftSetpoint(0);
                  m_leftMotorOk.setBoolean(Math.abs(m_leftShooterMotorEncoder.getVelocity()) < 5);
                },
                this)
            .withTimeout(ShooterConstants.TEST_TIMEOUT));
  }

  public void resetTestIndicators() {
    m_kickerMotorOk.setBoolean(false);
    m_leftMotorOk.setBoolean(false);
    m_rightMotorOk.setBoolean(false);
  }
}
