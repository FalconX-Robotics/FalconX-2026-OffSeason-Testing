package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Settings;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class KeepFromShooting extends Command {
    //run when trying to find the right speed for shooter when pressing the right trigger but not actually shoot
  private final Settings.OperatorSettings operatorSettings;
  private final Feeder feeder;
  private final Shooter shooter;

  public KeepFromShooting(RobotContainer robotContainer) {
    this.operatorSettings = robotContainer.settings.operatorSettings;
    this.feeder = robotContainer.subsystems.feeder;
    this.shooter = robotContainer.subsystems.shooter;

    addRequirements(this.feeder, this.shooter);
  }
    
  @Override
  public void execute() {
    // based on how far the trigger is pushed
    double value = this.operatorSettings.getRightTriggerAxis();
    this.feeder.motor.set(1.0);
    this.shooter.motor.set(value);
  }

  public void end(boolean interrupted){
    this.feeder.motor.set(0.0);
    this.shooter.motor.set(0.0);
  }
}
