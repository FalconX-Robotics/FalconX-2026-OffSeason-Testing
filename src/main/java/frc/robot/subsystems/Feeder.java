package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
  public final TalonFX motor = new TalonFX(Constants.ID.FEEDER_ID);
  final Shooter shooter;
  final CommandXboxController operatorController;
  
  public Feeder(RobotContainer robotContainer) {
    this.shooter = robotContainer.subsystems.shooter;
    this.operatorController = robotContainer.controllers.operator;

    /**
     * motor configs
     */
    final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = 400;
    talonFXConfigs.MotionMagic.MotionMagicJerk = 4000; // jerk is change in acceleration over time (like acceleration is to velocity, and velocity is to position)
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 60;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    final Slot1Configs slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = 0.25;
    slot1Configs.kV = 0.12;
    slot1Configs.kA = 0.01;
    slot1Configs.kP = 1.0;
    slot1Configs.kI = 0.0;
    slot1Configs.kD = 0.0;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  public void setFeederSpeed(double speed) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    motor.setControl(request.withVelocity(speed));
  }

  public void setFeederAcceleration(double acceleration) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    motor.setControl(request.withAcceleration(acceleration));
  }
  //making a method in feeder get a result from a method in another subsystem is unneeded --> just use shooter subsystem
  // public double getShooterSpeed() {
  //   return this.shooter.getShooterSpeed();
  // }

  public double getFeederSpeed() {
    return motor.getVelocity().getValueAsDouble();
  }

  public double getFeederAcceleration() {
    return motor.getAcceleration().getValueAsDouble();
  }
}
