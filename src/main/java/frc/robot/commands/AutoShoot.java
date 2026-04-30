package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

public class AutoShoot extends Command {
  // private static final Field2d field = new Field2d();

  private final Feeder feeder;
  private final Shooter shooter;
  private final SwerveSubsystem swerve;

    // private static final boolean isRedHub = true;

  private static final double offsetFactor = 180.0;
  private static final boolean invertOffset = true;

  private static Translation2d hubTranslation; // = new Translation2d(182.11 + (AutoShoot.isRedHub ? 287.0 : 0.0), 158.84);
  private static Rotation2d hubRotation; // = new Rotation2d(0.0, 0.0);
  private static Pose2d poseOfHub; // = new Pose2d(AutoShoot.hubTranslation, AutoShoot.hubRotation);

  public AutoShoot(RobotContainer robotContainer) {
    this.feeder = robotContainer.subsystems.feeder;
    this.shooter = robotContainer.subsystems.shooter;
    this.swerve = robotContainer.subsystems.swerve;

    addRequirements(feeder, shooter);

    hubRotation = new Rotation2d(0);
  }

  @Override
  public void initialize() {
    boolean isRedHub = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Red : false;
    hubTranslation = new Translation2d(182.11 + (isRedHub ? 287.0 : 0.0), 158.84);
    poseOfHub = new Pose2d(hubTranslation, AutoShoot.hubRotation);
    
    final Pose2d robotPos = swerve.getPose();

    final double a_sq = Math.pow(Units.metersToInches(robotPos.getX()) - poseOfHub.getX(), 2.0);
    // System.out.println(robotPos.getX() + " " + robotPos.getY());
    final double b_sq = Math.pow(Units.metersToInches(robotPos.getY()) - poseOfHub.getY(), 2.0);
    final double hypotenuse = Math.sqrt(a_sq + b_sq);
    final double power = Util.getPowerFromDistance(hypotenuse);

    // System.out.println("AUTOSHOOT POWER: " + power + "% ; CALCULATED HYPOTENUSE LENGTH: " + hypotenuse + "in");
    this.shooter.motor.set(power / 100.0);
    this.feeder.motor.set(-1.0);
  }

  public void end(boolean interrupted) {
    feeder.motor.set(0.0);
    shooter.motor.set(0.0);
  }
}
