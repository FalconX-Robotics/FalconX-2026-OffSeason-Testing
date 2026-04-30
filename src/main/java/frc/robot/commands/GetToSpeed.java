package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

public class GetToSpeed extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter shooter;
  private final Feeder feeder;
  private boolean isActive = false;
  private double velocity;
  private double velocityOfFeeder;

  public GetToSpeed(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;
    this.shooter = robotContainer.subsystems.shooter;
    this.feeder = robotContainer.subsystems.feeder;
    
    super.addRequirements(this.shooter, this.feeder);
  }
  
  public void initialize() {
    Vector2 targetPosition = Util.getTargetPosition();
    Vector2 robotPosition = new Vector2(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY());
    double distance = targetPosition.distance(robotPosition);

    // velocity = Util.findVelocity(distance);
    // shooter.setAutoShooterSpeed(velocity);

    
    shooter.motor.set(0.75);
    // velocityOfFeeder = shooter.getShooterSpeed() + 2;
    // feeder.setFeederSpeed(velocityOfFeeder);
    

    // velocityOfFeeder = shooter.motor.getVelocity().getValueAsDouble() + 2.0;
    // feeder.setFeederSpeed(velocityOfFeeder);

    feeder.motor.set(1);

    isActive = true;
  }


  public boolean isFinished() {
  //   if (isActive){
  //     boolean result = MathUtil.isNear(velocity, shooter.getSpeed(), 5);
  //     // System.out.println("velocity: " + velocity + " ; shooter spped: " + shooter.getSpeed());
  //     if (result){
  //       isActive = false;
  //     }
        
  //     return result;
  //   }

  
  //   return false;

  return false;
  }
  @Override
  public void execute() {
      // // System.out.println("GetToSpeed");
      // velocityOfFeeder = shooter.getShooterSpeed() + 2;
      // feeder.setFeederSpeed(velocityOfFeeder);

  }
  @Override
  public void end(boolean interrupted) {
      // System.out.println("GetToSpeed ended");
      shooter.recordShooterSpeed();
      shooter.motor.set(0);
      feeder.motor.set(0);
  }
}
