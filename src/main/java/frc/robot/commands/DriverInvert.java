package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriverInvert extends Command{
  private final RobotContainer robotContainer;

  public DriverInvert(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    this.robotContainer.settings.driverSettings.inverted = !this.robotContainer.settings.driverSettings.inverted;
  }
}
