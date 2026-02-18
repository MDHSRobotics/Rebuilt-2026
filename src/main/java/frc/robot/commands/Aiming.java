package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

/** This class provides instanced command factories for swerve drive aiming */
public class Aiming {
  private final CommandSwerveDrivetrain m_drivetrain;

  private final DoubleSupplier m_velocityXSupplier;
  private final DoubleSupplier m_velocityYSupplier;
  private final DoubleSupplier m_deadbandSupplier;

  // Joystick suppliers
  private final DoubleSupplier m_leftYSupplier;
  private final DoubleSupplier m_leftXSupplier;
  private final DoubleSupplier m_rightYSupplier;
  private final DoubleSupplier m_rightXSupplier;

  // NetworkTables
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final NetworkTable m_cameraTable = m_inst.getTable(VisionConstants.FRONT_LIMELIGHT_NAME);

  /**
   * Constructs an object that provides <a
   * href="https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#non-static-command-factories">instanced
   * command factories</a> for swerve drive aiming.
   *
   * @param drivetrain The drivetrain to drive and aim with.
   * @param velocityXSupplier A method reference or lambda that returns X velocity.
   * @param velocityYSupplierA method reference or lambda that returns Y velocity.
   * @param deadbandSupplier A method reference or lambda that returns deadband.
   * @param leftYSupplier A method reference or lambda that returns a left joystick's Y value.
   * @param leftXSupplier A method reference or lambda that returns a left joystick's X value.
   * @param rightYSupplier A method reference or lambda that returns a right joystick's Y value.
   * @param rightXSupplier A method reference or lambda that returns a right joystick's X value.
   */
  public Aiming(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier velocityXSupplier,
      DoubleSupplier velocityYSupplier,
      DoubleSupplier deadbandSupplier,
      DoubleSupplier leftYSupplier,
      DoubleSupplier leftXSupplier,
      DoubleSupplier rightYSupplier,
      DoubleSupplier rightXSupplier) {
    m_drivetrain = drivetrain;
    m_velocityXSupplier = velocityXSupplier;
    m_velocityYSupplier = velocityYSupplier;
    m_deadbandSupplier = deadbandSupplier;
    m_leftYSupplier = leftYSupplier;
    m_leftXSupplier = leftXSupplier;
    m_rightYSupplier = rightYSupplier;
    m_rightXSupplier = rightXSupplier;
  }
}
