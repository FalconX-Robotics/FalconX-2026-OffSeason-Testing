package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
  final Climber climberSubsystem;
  final TalonFX motor;
  

  /**
   * Extends the arm
   * @param robotContainer
   * */
  public ClimbDown(RobotContainer robotContainer) {
    this.climberSubsystem = robotContainer.subsystems.climber;
    this.motor = this.climberSubsystem.motor;
    addRequirements(this.climberSubsystem);
    motor.setPosition(0);
    SmartDashboard.putBoolean("Arm Up", false);
  }

  @Override
  public void initialize() {
    // motor.setPosition(0);
    // System.out.println("Rotations: " + motor.getPosition().getValueAsDouble());
  }

  @Override
  public void execute() {
    // System.out.println("Rotations: " + motor.getPosition().getValueAsDouble());
    this.climberSubsystem.motor.set(0.6);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.motor.set(0.0);
    SmartDashboard.putBoolean("Arm Up", true);

    System.out.println("ClimbDown ended");
    System.out.println("interrupeted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    final boolean result = Math.abs(motor.getPosition().getValueAsDouble()) >= 115.0;

    //TODO: remove later, if statement below is temp - sim won't end because it can't get motor position in sim
    if (Robot.isSimulation()) {
      if (Timer.getFPGATimestamp() > 20) {
        return true;
      }
    }
    // if (result) {
    //   // System.out.println("Rotations: " + motor.getPosition().getValueAsDouble());
    // }

    return result;
    // return false;
  }


}
