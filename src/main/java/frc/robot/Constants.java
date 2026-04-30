// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Util;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public abstract class Constants {
  public static final double ROBOT_MASS = Util.poundsToKilos(98.6);
  public static final Matter CHASSIS = new Matter((new Translation3d(0, 0, 0)), Constants.ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(15.0);
  public static final double MAX_ANGULAR_VELOCITY = 1 * Math.PI;
  public static final double CLAW_MASS = 0;
  public static final double CLAW_SPEED = 3;

  public static final double SHOOTER_ANGLE = 71;
  
  public static final double HEIGHT_OF_TARGET = 1.8288;

  public static final double SHOOTER_GEAR_RATIO = 1.0/1.0;

  public static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double RADIUS_OF_SHOOTER_WHEEL = Units.inchesToMeters(2.0);

  public static final Pose3d ROBOT_TO_CAMERA_POSE = new Pose3d(Units.inchesToMeters(4.5), Units.inchesToMeters(6.3), Units.inchesToMeters(20.275), new Rotation3d(0.0, Math.toRadians(5), Math.PI));
  public static final Pose3d ROBOT_TO_RPI_POSE = new Pose3d(Units.inchesToMeters(7.5), Units.inchesToMeters(9.0), Units.inchesToMeters(17.0), new Rotation3d(0.0, Math.toRadians(20.0), 0.0));

  public static final Pose2d STARTING_POSE = new Pose2d(3, 4, Rotation2d.fromDegrees(0.0));

  // LED is in PWM and thus doesn't need a unique value unless there are other things in PWM.
  // THIS IS AN IMPORT FROM 2024
  public static final int LED_PORT = 0;

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class SwerveDriveConstants {
    public static final double SPEED_MODE_SCALE = 0.2;
    
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class ID  {
    public static final int INTAKE_SHOOTER_ID = 3;
    public static final int FEEDER_ID = 1;
    public static final int CLIMBER_ID = 2;
    public static final int LIMIT_SWITCH_ID = 0; // Check this later
  }
  
  public static class GearRatio {
    //change when we know the REAL gear ratio
    public static final double ARM_GEAR_RATIO = 1.0/36.0;
  }
}

/**
 * PDH 1
 * BR Drive 10
 * BL Angle 17
 * BL Drive 16
 * FL Angle 15
 * FL Drive 14
 * BR Angle 11
 * FR Angle 13
 * FR Drive 12
 */
