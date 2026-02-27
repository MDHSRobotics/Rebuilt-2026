package frc.robot.subsystems.shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleEntry;
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

  private final RelativeEncoder m_shooterMotorEncoder = m_shooterLeftMotor.getEncoder();

  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_table = m_inst.getTable("Shooter");
  private final DoubleEntry m_shooterSpeedEntry = m_inst.getEntry(1);

  public Shooter(){
    
  }
}
