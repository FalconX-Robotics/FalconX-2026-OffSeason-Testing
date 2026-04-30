package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.Vision;

public class ToggleVision extends Command {
    private final Vision vision;

    public ToggleVision(RobotContainer robotContainer) {
        this.vision = robotContainer.subsystems.swerve.getVision();
    }

    @Override
    public void initialize() {
        this.vision.toggleDisabledState();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
