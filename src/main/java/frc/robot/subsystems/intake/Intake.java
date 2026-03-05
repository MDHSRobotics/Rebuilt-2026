package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // Motors
  private final SparkFlex m_intakeRightMotor =
      new SparkFlex(IntakeConstants.INTAKE_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_intakeLeftMotor =
      new SparkFlex(IntakeConstants.INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_spinnerMotor =
      new SparkFlex(IntakeConstants.INTAKE_SPINNERS_MOTOR_ID, MotorType.kBrushless);

  // PID Controllers
  private final SparkClosedLoopController m_intakeController =
      m_intakeLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_spinnerController =
      m_spinnerMotor.getClosedLoopController();

  // Encoders
  private final AbsoluteEncoder m_intakeLeftEncoder = m_intakeLeftMotor.getAbsoluteEncoder();
  private final AbsoluteEncoder m_spinnerEncoder = m_spinnerMotor.getAbsoluteEncoder();

  // Networktables data
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Intake");
  private final DoublePublisher m_intakePositionPub =
      m_inst.getDoubleTopic("Intake Current Position ").publish();
  private final DoublePublisher m_spinnerSpeedPub =
      m_inst.getDoubleTopic("Spinner Motors Speed ").publish();
  private final DoublePublisher m_targetPositionPub =
      m_table.getDoubleTopic("Target Position (Rotations)").publish();
  private final DoublePublisher m_targetSpeedPub =
      m_table.getDoubleTopic("Target Speed (RPM)").publish();

  public Intake() {
    SparkFlexConfig intakeRightConfig = new SparkFlexConfig();
    intakeRightConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    intakeRightConfig
        .signals
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true);
    intakeRightConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(IntakeConstants.K_P);
    m_intakeRightMotor.configure(
        intakeRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeLeftConfig = new SparkFlexConfig();
    intakeLeftConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    intakeLeftConfig
        .signals
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true);
    intakeLeftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(IntakeConstants.K_P);
    m_intakeLeftMotor.configure(
        intakeLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeSpinnerConfig = new SparkFlexConfig();
    intakeSpinnerConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_spinnerMotor.configure(
        intakeSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    double position = m_intakeLeftEncoder.getPosition();
    double velocity = m_spinnerEncoder.getVelocity();
    m_intakePositionPub.set(position);
    m_spinnerSpeedPub.set(velocity);
  }

  public void disableMotors() {
    m_intakeRightMotor.stopMotor();
    m_intakeLeftMotor.stopMotor();
    m_spinnerMotor.stopMotor();
  }

  public void runMotor(
      double leftIntakeMotorPower, double rightIntakeMotorPower, double spinnerMotorPower) {
    m_intakeRightMotor.set(leftIntakeMotorPower);
    m_intakeLeftMotor.set(rightIntakeMotorPower);
    m_spinnerMotor.set(rightIntakeMotorPower);
  }

  public void intakeBall() {
    m_intakeController.setSetpoint(IntakeConstants.PICKUP_POSITION, ControlType.kPosition);
    m_spinnerController.setSetpoint(IntakeConstants.INTAKE_SPINNERS_SPEED, ControlType.kVelocity);
    m_intakePositionPub.set(m_intakeLeftEncoder.getPosition());
    m_spinnerSpeedPub.set(m_spinnerEncoder.getVelocity());
    m_targetPositionPub.set(IntakeConstants.PICKUP_POSITION);
    m_targetSpeedPub.set(IntakeConstants.INTAKE_SPINNERS_SPEED);
  }

  public void stowedIntake() {
    m_intakeController.setSetpoint(IntakeConstants.STOWED_POSITION, ControlType.kPosition);
    m_spinnerMotor.stopMotor();
    m_intakePositionPub.set(m_intakeLeftEncoder.getPosition());
    m_spinnerSpeedPub.set(m_spinnerEncoder.getVelocity());
    m_targetPositionPub.set(IntakeConstants.STOWED_POSITION);
    m_targetSpeedPub.set(0);
  }
}
