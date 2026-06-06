package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class Autos extends Command {

    RobotContainer robotContainer; 
    AutoFactory autoFactory;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.autoFactory = this.robotContainer.autoFactory;

        SmartDashboard.putBoolean("Marker Auto hit", false);
    }
    public AutoRoutine middleShoot() {
        
        AutoRoutine routine = this.autoFactory.newRoutine("middleShoot");

        AutoTrajectory middleToShoot = routine.trajectory("middleToShoot");

        routine.active().onTrue(

        Commands.sequence(
            middleToShoot.resetOdometry(),
            Commands.race(
                middleToShoot.cmd(),
                this.robotContainer.commands.medPowGetToSpeed
                
            )
          
        )

        );

        middleToShoot.done().onTrue(
            Commands.race(
                new WaitCommand(5),
                this.robotContainer.commands.meduimPowerControlledShoot

            )
            
            );

        middleToShoot.done().onTrue(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Marker Auto hit", false);
        }));

        middleToShoot.atTime("Marker").onTrue(Commands.runOnce(() -> {
            SmartDashboard.putBoolean("Marker Auto hit", true); 
        }));

        return routine;

    }
}
