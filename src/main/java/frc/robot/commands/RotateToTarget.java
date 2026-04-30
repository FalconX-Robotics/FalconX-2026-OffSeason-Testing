package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.Util;

public class RotateToTarget extends Command{
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController pidController;
  private final double MAX_SPEED = 2;

  double currentAngle;
  double targetAngle;

  public RotateToTarget(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;
    this.pidController = new PIDController(3, 0.0, 0.0);

    super.addRequirements(this.swerveSubsystem);
  }

  public void recalculateAngle() {
    Vector2 targetPosition = Util.getTargetPosition();
    double robotX = Units.metersToInches(swerveSubsystem.getPose().getX());
    double robotY = Units.metersToInches(swerveSubsystem.getPose().getY());

    targetAngle = Math.atan2(targetPosition.y-robotY, targetPosition.x-robotX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recalculateAngle();
    // System.out.println(targetAngle);
    targetAngle -= Math.PI/2;
    pidController.setSetpoint(targetAngle);
    final Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "TARGET ANGLE:", "targetAngle: " + targetAngle);
    Elastic.sendNotification(notification);
    SmartDashboard.putNumber("Current Angle", currentAngle);
    SmartDashboard.putNumber("Target Angle", targetAngle - Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveSubsystem.getYaw().getRadians();
    
    double numberofRotations = swerveSubsystem.getYaw().getRotations() * Math.PI * 2;
    
    double remapvalue = (numberofRotations - targetAngle) % Math.PI * 2;
    
    if (remapvalue < 0) {
      remapvalue += Math.PI * 2;
    }
    
    remapvalue = (remapvalue - Math.PI + targetAngle);
    
    double clamp = MathUtil.clamp(pidController.calculate(remapvalue), -MAX_SPEED, MAX_SPEED);
    swerveSubsystem.drive(new ChassisSpeeds(0,0, clamp));
    
    SmartDashboard.putNumber("Clamp", clamp);
    
    SmartDashboard.putNumber("Remap value", remapvalue);
    SmartDashboard.putNumber("Target Angle", targetAngle);
    
    // System.out.println("RotateToTarget");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("RotateToTarget ended");
    final Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "CURRENT ANGLE:", "Current angle" + currentAngle);
    Elastic.sendNotification(notification);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double difference = targetAngle - currentAngle;
    double targetAngleRange = Math.toRadians(2);

    boolean angleAtTarget = Math.abs(difference) < targetAngleRange;

    double rotationSpeed = Math.toDegrees(swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

    double targetSpeedRange = 5;
    boolean speedAtTarget = Math.abs(rotationSpeed) < targetSpeedRange;

    return angleAtTarget && speedAtTarget;
  }
}
