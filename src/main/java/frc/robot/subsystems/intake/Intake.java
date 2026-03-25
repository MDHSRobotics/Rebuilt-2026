package frc.robot.subsystems.intake;

import com.revrobotics.AbsoluteEncoder;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Testable;

public class Intake extends SubsystemBase implements Testable {
  // Motors
  private final SparkFlex m_intakeRightMotor =
      new SparkFlex(IntakeConstants.INTAKE_RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_intakeLeftMotor =
      new SparkFlex(IntakeConstants.INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_spinnerMotor =
      new SparkFlex(IntakeConstants.INTAKE_SPINNERS_MOTOR_ID, MotorType.kBrushless);

  // PID Controllers
  private final SparkClosedLoopController m_intakeLeftController =
      m_intakeLeftMotor.getClosedLoopController();
  private final SparkClosedLoopController m_intakeRightController =
      m_intakeRightMotor.getClosedLoopController();
  private final SparkClosedLoopController m_spinnerController =
      m_spinnerMotor.getClosedLoopController();

  // Encoders
  private final AbsoluteEncoder m_intakeLeftEncoder = m_intakeLeftMotor.getAbsoluteEncoder();
  private final AbsoluteEncoder m_intakeRightEncoder = m_intakeRightMotor.getAbsoluteEncoder();
  private final RelativeEncoder m_spinnerEncoder = m_spinnerMotor.getEncoder();

  // Networktables data
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Intake");
  private final DoublePublisher m_leftCurrentPositionPub =
      m_table.getDoubleTopic("Left Intake Current Position ").publish();
  private final DoublePublisher m_rightCurrentPositionPub =
      m_table.getDoubleTopic("Right Intake Current Position ").publish();
  private final DoublePublisher m_leftTargetPositionPub =
      m_table.getDoubleTopic("Left Target Position (Rotations)").publish();
  private final DoublePublisher m_rightTargetPositionPub =
      m_table.getDoubleTopic("Right Target Position (Rotations)").publish();
  private final DoublePublisher m_spinnerCurrentVelocityPub =
      m_table.getDoubleTopic("Spinner Motor Velocity ").publish();
  private final DoublePublisher m_spinnerTargetSpeedPub =
      m_table.getDoubleTopic("Spinner Target Speed (RPM)").publish();

  private final NetworkTableEntry m_spinnerMotorOk =
      m_inst.getTable("Test").getEntry("IntakeSpinnerMotorRPM_OK");
  private final NetworkTableEntry m_leftMotorOk =
      m_inst.getTable("Test").getEntry("IntakeLeftMotorRPM_OK");
  private final NetworkTableEntry m_rightMotorOk =
      m_inst.getTable("Test").getEntry("IntakeRightMotorRPM_OK");

  private boolean deployed = false;

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
    intakeSpinnerConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kCoast);
    m_spinnerMotor.configure(
        intakeSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    /* Logging */
    m_leftCurrentPositionPub.set(m_intakeLeftEncoder.getPosition());
    m_rightCurrentPositionPub.set(m_intakeRightEncoder.getPosition());
    m_spinnerTargetSpeedPub.set(m_spinnerMotor.get());
    m_spinnerCurrentVelocityPub.set(m_spinnerEncoder.getVelocity());
  }

  public void stopMotors() {
    m_intakeRightMotor.stopMotor();
    m_intakeLeftMotor.stopMotor();
    m_spinnerMotor.stopMotor();
  }

  public void runMotors(
      double leftIntakeMotorPower, double rightIntakeMotorPower, double spinnerMotorPower) {
    m_intakeRightMotor.set(leftIntakeMotorPower);
    m_intakeLeftMotor.set(rightIntakeMotorPower);
    m_spinnerMotor.set(rightIntakeMotorPower);
  }

  public void runMotors(double leftIntakeMotorPower) {
    m_intakeLeftMotor.set(leftIntakeMotorPower);
    m_intakeRightMotor.set(leftIntakeMotorPower);
  }

  public void runSpinner(double power) {
    m_spinnerMotor.set(power);
  }

  public void deployedPosition() {
    deployed = true;
    m_intakeLeftController.setSetpoint(IntakeConstants.PICKUP_POSITION_LEFT, ControlType.kPosition);
    m_intakeRightController.setSetpoint(
        IntakeConstants.PICKUP_POSITION_RIGHT, ControlType.kPosition);
    m_leftCurrentPositionPub.set(m_intakeLeftEncoder.getPosition());
    m_rightCurrentPositionPub.set(m_intakeRightEncoder.getPosition());
    m_leftTargetPositionPub.set(IntakeConstants.PICKUP_POSITION_LEFT);
    m_rightTargetPositionPub.set(IntakeConstants.PICKUP_POSITION_RIGHT);
  }

  public void stowedPosition() {
    deployed = false;
    m_intakeLeftController.setSetpoint(IntakeConstants.STOWED_POSITION_LEFT, ControlType.kPosition);
    m_intakeRightController.setSetpoint(
        IntakeConstants.STOWED_POSITION_RIGHT, ControlType.kPosition);
    m_leftCurrentPositionPub.set(m_intakeLeftEncoder.getPosition());
    m_rightCurrentPositionPub.set(m_intakeRightEncoder.getPosition());
    m_leftTargetPositionPub.set(IntakeConstants.STOWED_POSITION_LEFT);
    m_rightTargetPositionPub.set(IntakeConstants.STOWED_POSITION_RIGHT);
  }

  public void stopSpinnerMotors() {
    m_spinnerMotor.stopMotor();
  }

  public boolean isDeployed() {
    return deployed;
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_spinnerMotor.set(IntakeConstants.TEST_POWER);
                  double rpm = m_spinnerEncoder.getVelocity();
                  m_spinnerMotorOk.setBoolean(rpm > IntakeConstants.TEST_RPM);
                },
                this)
            .withTimeout(IntakeConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_spinnerMotor.set(0.0);
                  m_spinnerMotorOk.setBoolean(Math.abs(m_spinnerEncoder.getVelocity()) < 5);
                },
                this)
            .withTimeout(IntakeConstants.TEST_TIMEOUT),
        Commands.run(() -> deployedPosition(), this));
  }

  public void resetTestIndicators() {
    m_spinnerMotorOk.setBoolean(false);
    m_leftMotorOk.setBoolean(false);
    m_rightMotorOk.setBoolean(false);
  }
}
