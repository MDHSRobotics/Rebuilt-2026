package frc.robot.subsystems.hopper;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final SparkFlex m_hopperMotor =
      new SparkFlex(HopperConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder m_hopperEncoder = m_hopperMotor.getEncoder();

  // Networktables data
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Hopper");
  private final DoublePublisher m_hopperSpeedPub =
      m_inst.getDoubleTopic("Hopper Current Speed ").publish();

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
}
