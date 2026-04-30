package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class Intake extends Command {
  private final Feeder feeder;
  private final Shooter shooter;

  public Intake(RobotContainer robotContainer) {
    this.feeder = robotContainer.subsystems.feeder;
    this.shooter = robotContainer.subsystems.shooter;

    super.addRequirements(this.feeder, this.shooter);
  }

  @Override
  public void execute() {
    this.feeder.motor.set(1.0);
    this.shooter.motor.set(0.9);
  }

  public void end(boolean interrupted){
    this.feeder.motor.set(0.0);
    this.shooter.motor.set(0.0);
  }
}
