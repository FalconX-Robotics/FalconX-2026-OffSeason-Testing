package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SwitchVisionState extends Command {

    RobotContainer robotContainer;

    public SwitchVisionState(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        // this.robotContainer.subsystems.swerve.useVision = !this.robotContainer.subsystems.swerve.useVision;
    }
}
