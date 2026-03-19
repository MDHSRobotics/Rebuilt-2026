package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;

public interface Testable {
  Command test();

  void resetTestIndicators();
}
