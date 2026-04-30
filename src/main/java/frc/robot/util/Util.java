package frc.robot.util;

import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Robot;

public class Util {
  private static LocalDateTime startTime;

  public static String getLogFilename() {
    final DateTimeFormatter timeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(ZoneId.of("UTC"));
    final String prefix = Robot.isSimulation() ? "sim_" : "robot_";
    final String suffix = DriverStation.isFMSAttached() ? "_match" + DriverStation.getMatchNumber() : "_test" + (DriverStation.isAutonomous() ? "Auto" : "Teleop");
    
    return prefix + timeFormatter.format(Util.startTime) + suffix + ".wpilog";
  }

  public static void setStartTime(LocalDateTime time) {
    Util.startTime = time;
  }

  public static DoubleLogEntry createDoubleLog(String name) {
    return new DoubleLogEntry(DataLogManager.getLog(), name);
  }

  public static IntegerLogEntry createIntLog(String name) {
    return new IntegerLogEntry(DataLogManager.getLog(), name);
  }

  public static BooleanLogEntry createBooleanLog(String name) {
    return new BooleanLogEntry(DataLogManager.getLog(), name);
  }

  public static double poundsToKilos(double pounds) {
    return pounds * 0.453592;
  }

  public static double remap(double in, double lowIn, double hiIn, double lowOut, double hiOut) {
    return lowOut + (in - lowIn) * (hiOut - lowIn) / (hiIn - lowIn);
  }

  public static Pose3d transformToPose(Transform3d transform) {
    return new Pose3d(transform.getTranslation(), transform.getRotation());
  }

  public static Vector2 getTargetPosition() {
    final double x = (DriverStation.getAlliance().get() == Alliance.Red) ? (651 - 182.11) : 182.11;
    final double y = 317.0 / 2.0;
    return new Vector2(x, y);
  }

  public static double findVelocity(double distanceInMeters) {
    final double angle = Math.toRadians(Constants.SHOOTER_ANGLE);

    /*
    adjacent is base ___
    opposite is height |
    hypotenus is sloped /
     */

    final double cos = Math.cos(angle); //adjacent over hypotenus
    final double tan = Math.tan(angle); // opposite over adjacent

    final double x = distanceInMeters; //adjacent
    final double y = Constants.HEIGHT_OF_TARGET; //opposite

    final double a = y - (tan * x); // returns 0 --> opposite - (opposite/adjacent * adjacent)
    final double c = 4.9 * Math.pow(x, 2) / Math.pow(cos, 2);

    return Math.abs(Math.sqrt(-4 * a * c) / (2 * a));
  }

  public static double getPowerFromDistance(double distanceInInches) {
    // https://www.desmos.com/calculator/rz9wefetyr

    final double factor = 82.5 / 264.5;
    return (factor * distanceInInches) + 0.03;
  }
}