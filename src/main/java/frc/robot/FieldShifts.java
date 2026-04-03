package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldShifts {
  private static final double transitionShiftEnd = 130;
  private static final double shift1End = 105;
  private static final double shift2End = 80;
  private static final double shift3End = 55;
  private static final double shift4End = 30;

  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean oddShiftActive = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    switch (getShift()) {
      case 1:
      case 3:
        return oddShiftActive;
      case 2:
      case 4:
        return !oddShiftActive;
      default:
        // Transition or end game
        return true;
    }
  }

  public static double remainingShiftTime() {
    double matchTime = DriverStation.getMatchTime();

    double endTime = switch (getShift()) {
      case 0 -> transitionShiftEnd;
      case 1 -> shift1End;
      case 2 -> shift2End;
      case 3 -> shift3End;
      case 4 -> shift4End;
      default -> 0;
    };

    return matchTime - endTime;
  }

  private static int getShift() {
    if (!DriverStation.isTeleopEnabled()) {
      return -1;
    }

    double matchTime = DriverStation.getMatchTime();

    if (matchTime > transitionShiftEnd) {
      // Transition shift
      return 0;
    } else if (matchTime > shift1End) {
      // Shift 1
      return 1;
    } else if (matchTime > shift2End) {
      // Shift 2
      return 2;
    } else if (matchTime > shift3End) {
      // Shift 3
      return 3;
    } else if (matchTime > shift4End) {
      // Shift 4
      return 4;
    } else {
      // End game
      return 5;
    }
  }
}
