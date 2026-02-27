package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  private final SparkFlex m_intakeTopMotor =
      new SparkFlex(IntakeConstants.INTAKE_TOP_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_intakeBottomMotor =
      new SparkFlex(IntakeConstants.INTAKE_BOTTOM_MOTOR_ID, MotorType.kBrushless);
  private final SparkFlex m_spinnerMotor =
      new SparkFlex(IntakeConstants.INTAKE_SPINNERS_MOTOR_ID, MotorType.kBrushless);

  private final RelativeEncoder m_intakeTopMotorEncoder = m_intakeTopMotor.getEncoder();

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Intake");

  public Intake() {
    SparkFlexConfig intakeTopConfig = new SparkFlexConfig();
    intakeTopConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_intakeTopMotor.configure(
        intakeTopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeBottomConfig = new SparkFlexConfig();
    intakeBottomConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_intakeBottomMotor.configure(
        intakeBottomConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig intakeSpinnerConfig = new SparkFlexConfig();
    intakeSpinnerConfig.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_spinnerMotor.configure(
        intakeSpinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command disableMotorsCommand() {
    return this.runOnce(
            () -> {
              m_intakeTopMotor.stopMotor();
              m_intakeBottomMotor.stopMotor();
              m_spinnerMotor.stopMotor();
            })
        .andThen(Commands.idle(this));
  }

  public Command setPowerCommand(
      DoubleSupplier topIntakeMotorPower,
      DoubleSupplier bottomIntakeMotorPower,
      DoubleSupplier spinnerMotorPower) {
    return this.run(
        () -> {
          m_intakeTopMotor.set(topIntakeMotorPower.getAsDouble());
          m_intakeBottomMotor.set(bottomIntakeMotorPower.getAsDouble());
          m_spinnerMotor.set(bottomIntakeMotorPower.getAsDouble());
        });
  }
}
