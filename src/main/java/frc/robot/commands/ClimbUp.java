package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
  final TalonFX motor;
  final Climber climberSubsystem;


  //arm down
  public ClimbUp(RobotContainer robotContainer) {
    this.climberSubsystem = robotContainer.subsystems.climber;
    this.motor = this.climberSubsystem.motor;

    super.addRequirements(this.climberSubsystem);
  }
  @Override
  public void initialize() {
      // System.out.println("Rotations: " + motor.getPosition().getValueAsDouble());
  }

  @Override
  public void execute() {
    motor.set(-0.3);
  }

  @Override
  public boolean isFinished() {
    return climberSubsystem.ClimbUpDone();
  }

  @Override
  public void end(boolean interrupted) {
    motor.set(0.0);
    // System.out.println("ClimbUp ended");
  }
}
