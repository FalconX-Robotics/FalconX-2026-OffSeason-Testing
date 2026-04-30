package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {
  public final TalonFX motor = new TalonFX(Constants.ID.INTAKE_SHOOTER_ID);
 
  final SwerveSubsystem swerveSubsystem;
  final CommandXboxController operatorController; 

  double lastCommandedShooterSpeed;

  public Shooter(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;
    this.operatorController = robotContainer.controllers.operator;

    final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = 400;
    talonFXConfigs.MotionMagic.MotionMagicJerk = 4000; // jerk is change in acceleration over time (like acceleration is to velocity, and velocity is to position)
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    final Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25;
    slot0Configs.kV = 0.12;
    slot0Configs.kA = 0.01;
    slot0Configs.kP = 1.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    motor.getConfigurator().apply(talonFXConfigs);
  }
  
  public void setAutoShooterSpeed(double speed) {
    speed /= Constants.SHOOTER_WHEEL_RADIUS;
    speed *= Constants.SHOOTER_GEAR_RATIO;
    speed /= 2 * Math.PI;
    
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    motor.setControl(request.withVelocity(speed));      
  }

  public void setShooterSpeed(double speed) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    motor.setControl(request.withVelocity(speed));     
  }

  public double getShooterSpeed() {
    return motor.getVelocity().getValueAsDouble();
  }

  public void recordShooterSpeed() {
    lastCommandedShooterSpeed = motor.getVelocity().getValueAsDouble();
  }

  public double getLastReccordedShooterVelocity() {
    return lastCommandedShooterSpeed;
  }

  public double getSpeed() {
    double velocity = motor.getVelocity().getValueAsDouble() * (2.0 * Math.PI) / Constants.SHOOTER_GEAR_RATIO; // velocity of wheels

    return velocity * Constants.SHOOTER_WHEEL_RADIUS; // velocity at which objects comes out
  }

  
  public void setShooterAcceleration(double acceleration) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    motor.setControl(request.withAcceleration(acceleration));
  }

  public boolean isHolding() {
    // TODO - need prox sensor to check if ball is being held
    return false;
  }
  // public void recordLastCommandedShooterSpeed() {
  //   lastCommandedShooterSpeed = motor.get();
  //   // System.out.print(lastCommandedShooterSpeed);
  // }
  // public double getLastRecordedCommandedShooterSpeed() {
  //   return lastCommandedShooterSpeed;
  // }
}