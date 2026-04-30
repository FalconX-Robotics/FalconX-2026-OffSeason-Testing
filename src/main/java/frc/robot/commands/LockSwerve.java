package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class LockSwerve extends Command {
    RobotContainer robotContainer;
    SwerveSubsystem swerveSubsystem;

    public LockSwerve(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.swerveSubsystem = this.robotContainer.subsystems.swerve;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.lock();
    }

}