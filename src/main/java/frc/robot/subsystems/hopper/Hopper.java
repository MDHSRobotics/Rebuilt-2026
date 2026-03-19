package frc.robot.subsystems.hopper;

import static frc.robot.util.EpsilonEquals.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
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

public class Hopper extends SubsystemBase implements Testable {
  private final SparkFlex m_hopperMotor =
      new SparkFlex(HopperConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_hopperEncoder = m_hopperMotor.getEncoder();

  // Networktables data
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Hopper");
  private final DoublePublisher m_hopperSpeedPub =
      m_table.getDoubleTopic("Hopper Current Speed ").publish();

  private final NetworkTableEntry m_hopperMotorOk =
      m_inst.getTable("Test").getEntry("HopperMotorRPM_OK");

  public Hopper() {
    SparkFlexConfig m_hopperConfig = new SparkFlexConfig();
    m_hopperConfig.smartCurrentLimit(HopperConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake);
    m_hopperConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(HopperConstants.K_P);
    m_hopperMotor.configure(
        m_hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runHopper(double speed) {
    m_hopperMotor.set(speed);
    m_hopperSpeedPub.set(m_hopperEncoder.getVelocity());
  }

  public void stopMotors() {
    m_hopperMotor.stopMotor();
  }

  public Command test() {
    return Commands.sequence(
        Commands.run(
                () -> {
                  m_hopperMotor.set(HopperConstants.TEST_POWER);
                  double rpm = m_hopperEncoder.getVelocity();
                  m_hopperMotorOk.setBoolean(rpm > HopperConstants.TEST_RPM);
                },
                this)
            .withTimeout(HopperConstants.TEST_TIMEOUT),
        Commands.run(
                () -> {
                  m_hopperMotor.set(0.0);
                  m_hopperMotorOk.setBoolean(
                      epsilonEquals(m_hopperEncoder.getVelocity(), 0.0, 5.0));
                },
                this)
            .withTimeout(HopperConstants.TEST_TIMEOUT));
  }

  public void resetTestIndicators() {
    m_hopperMotorOk.setBoolean(false);
  }
}
