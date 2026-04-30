package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ControlledGetToSpeed extends Command {
  private final Shooter shooterSubsystem;
  private final Feeder feederSubsystem;
  private final double power;

  public ControlledGetToSpeed(RobotContainer robotContainer, double power) {
    this.shooterSubsystem = robotContainer.subsystems.shooter;
    this.feederSubsystem = robotContainer.subsystems.feeder;
    this.power = power;

    super.addRequirements(this.shooterSubsystem, this.feederSubsystem);
  }

  @Override
  public void initialize() {
    this.shooterSubsystem.motor.set(this.power);
  }

  @Override
  public void end(boolean interrupted) {
    // this.shooterSubsystem.motor.set(0.0);
  }
}
