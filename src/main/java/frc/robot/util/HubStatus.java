package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class HubStatus {

  public static boolean isHubActive() {
    return isHubActive(0, 0);
  }

  /** This method is used to determine if the Hub is active including a pre and post time */
  public static boolean isHubActive(double pre, double post) {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shift1Active =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    // Shift time windows: each entry is {start, end} in match time (counting down).
    // Regions outside these windows (>130 and <30) are always active.
    double[][] shiftWindows = {
      {130, 105}, // Shift 1
      {105, 80}, // Shift 2
      {80, 55}, // Shift 3
      {55, 30}, // Shift 4
    };

    // Shift 1 starts with shift1Active, then alternates each shift.
    boolean[] shiftActive = {
      shift1Active, !shift1Active, shift1Active, !shift1Active,
    };

    if (matchTime > 130 || matchTime <= 30) {
      return true;
    }

    for (int i = 0; i < shiftWindows.length; i++) {
      double shiftStart = shiftWindows[i][0]; // higher match time = earlier in match
      double shiftEnd = shiftWindows[i][1];

      // A shift is "on" for [shiftEnd, shiftStart], counting down.
      // With pre/post: active if matchTime is within (shiftStart + pre) down to (shiftEnd - post).
      if (matchTime <= shiftStart + pre && matchTime >= shiftEnd - post) {
        return shiftActive[i];
      }
    }

    // Fallback (shouldn't be reached given the windows cover 30–130).
    return true;
  }
}
