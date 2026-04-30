package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class JiggleRobot extends Command {
  private final SwerveSubsystem swerveSubsystem;
  double forwardVelocity;
  double sidewaysVelocity;
  double rotationalvelocity;

  public JiggleRobot(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;
  }

  @Override
  public void execute() {
    recalcuateForwardVelocity();
    recalcuateSidewaysVelocity();
    // recalculateRotationalVelocity();
    swerveSubsystem.drive(new ChassisSpeeds(forwardVelocity, sidewaysVelocity, 0));
  }

  private void recalcuateForwardVelocity(){
    double min = Units.inchesToMeters(-3);
    double max = Units.inchesToMeters(3);
    forwardVelocity = Math.random() * (max - min) + min;
  }

  private void recalcuateSidewaysVelocity(){
    double min = Units.inchesToMeters(-3);
    double max = Units.inchesToMeters(3);
    sidewaysVelocity = Math.random() * (max - min) + min;
  }

  // private void recalculateRotationalVelocity() {
  //     double min = -Math.PI;
  //     double max = Math.PI;
  //     rotationalvelocity = Math.random() * (max - min) + min;
  // }
}
