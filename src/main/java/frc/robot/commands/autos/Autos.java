package frc.robot.commands.autos;

import java.util.function.BooleanSupplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ControlledGetToSpeed;
import frc.robot.commands.ControlledShoot;

public class Autos extends Command {

    RobotContainer robotContainer; 
    AutoFactory autoFactory;

    public Autos(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.autoFactory = this.robotContainer.autoFactory;

        SmartDashboard.putBoolean("Marker Auto hit", false);
    }
    /**
     * AutoRoutine to go from middle hub position (facing hub) to shooting position near climb
     * @return AutoRoutinue
     */
    public AutoRoutine middleShoot() {
        
        AutoRoutine routine = this.autoFactory.newRoutine("middleShoot");

        AutoTrajectory middleToShoot = routine.trajectory("middleToShoot");

        routine.active().onTrue(

        Commands.sequence(
            middleToShoot.resetOdometry(),
            Commands.race(
                middleToShoot.cmd(),
                new ControlledGetToSpeed(this.robotContainer, 0.85) //new instance to prevent illegal argument error
                
            )
          
        )

        );

        middleToShoot.done().onTrue(
            Commands.race(
                new WaitCommand(5),
                new ControlledShoot(this.robotContainer, 0.70)

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

    /**
     * Middle Hub Pose to Shooting Pose to Right Trench Pose Auto Routinue
     * @return AutoRoutine
     */
    public AutoRoutine mToShootToR() {
        AutoRoutine routine = this.autoFactory.newRoutine("mToShootToR");

        AutoTrajectory lineUpToShootTraj = routine.trajectory("mToShootToR", 0);
        AutoTrajectory toRightFromShootTraj = routine.trajectory("mToShootToR", 1);

        routine.active().onTrue(
            Commands.sequence(
                lineUpToShootTraj.resetOdometry(),
            Commands.race(
                new ControlledGetToSpeed(this.robotContainer, 0.85),      
                lineUpToShootTraj.cmd()
            )
            
            )
        );
            lineUpToShootTraj.done().onTrue(
               Commands.sequence(
                Commands.runOnce( () -> SmartDashboard.putBoolean("Marker Auto hit", true)),
                Commands.race(
                    new ControlledShoot(robotContainer, 0.70),
                    new WaitCommand(4)
                ),
                Commands.runOnce( () -> SmartDashboard.putBoolean("Marker Auto hit", false)),
                 toRightFromShootTraj.cmd()
               )
            );

            return routine;


    }

    public AutoRoutine mToClimb() {
        AutoRoutine routine = this.autoFactory.newRoutine("mToClimb");

        AutoTrajectory toClimbReadyTraj = routine.trajectory("mToClimb", 0);
        AutoTrajectory toClimbTraj = routine.trajectory("mToClimb", 1);
        
        ClimbDown climbDown = new ClimbDown(this.robotContainer);
        routine.active().onTrue(
            Commands.sequence(
                toClimbReadyTraj.resetOdometry(),
                toClimbReadyTraj.cmd()
            ) 
        );

        toClimbReadyTraj.active().onTrue(
            climbDown
        );

        toClimbReadyTraj.done().onTrue(
        Commands.waitUntil(() -> !(climbDown.isScheduled())).andThen(toClimbTraj.cmd())
        );

        toClimbTraj.done().onTrue(
            new ClimbUp(robotContainer)
        );

        //trying out .atTime()
        toClimbReadyTraj.atTime(0.8).onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Marker Auto hit", true))
        );

        
        toClimbTraj.atTime(1.0).onTrue(
            Commands.runOnce(() -> SmartDashboard.putBoolean("Marker Auto hit", false))
        );
        ////////

        return routine;
    }


}
