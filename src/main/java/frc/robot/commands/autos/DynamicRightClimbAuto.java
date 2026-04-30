package frc.robot.commands.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class DynamicRightClimbAuto extends Command {

    PathPlannerAuto auto;
    PathPlannerPath path;
    // Waypoint startWaypoint;
    // Waypoint waypoint2;
    // Waypoint endWaypoint;
    PathConstraints pathConstraints;
    RobotContainer robotContainer;


    public DynamicRightClimbAuto(RobotContainer robotContainer) {
        // startWaypoint = new Waypoint(null, new Translation2d(2, 2), null);
        // waypoint2 = new Waypoint(null, null, null);
        // endWaypoint = new Waypoint(null, null, null);

        this.robotContainer = robotContainer;

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(3.63796005706134, 0.7809272467902997, Rotation2d.fromDegrees(180)),
        new Pose2d(1.322211111111111, 3.246, Rotation2d.fromDegrees(180)),
        new Pose2d(0.38674444444444445, 3.3723777777777775, Rotation2d.fromDegrees(180))
);        

        pathConstraints = this.robotContainer.subsystems.swerve.pathConstraints;

        path = new PathPlannerPath(waypoints,
                pathConstraints,
                 new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                 new GoalEndState(0, Rotation2d.fromDegrees(180)));
        
        path.preventFlipping = DriverStation.getAlliance().get() == Alliance.Blue;

        Command runpath = AutoBuilder.followPath(path);
        SequentialCommandGroup sequentialCommandGroup = new SequentialCommandGroup(this.robotContainer.commands.climbDown, runpath, this.robotContainer.commands.climbUp);
        
        auto = new PathPlannerAuto(sequentialCommandGroup);

        
    }
    
}