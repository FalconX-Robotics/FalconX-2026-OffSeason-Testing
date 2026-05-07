package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Autos extends Command {

    RobotContainer robotContainer; 
    AutoFactory autoFactory;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.autoFactory = this.robotContainer.autoFactory;
    }
    public AutoRoutine middleShoot() {
        
        AutoRoutine routine = this.autoFactory.newRoutine("middleShoot");

        AutoTrajectory middleToShoot = routine.trajectory("middleToShoot");

        routine.active().onTrue(

        Commands.sequence(
            middleToShoot.resetOdometry(),
            middleToShoot.cmd()
        )

        );

        return routine;

    }
}
