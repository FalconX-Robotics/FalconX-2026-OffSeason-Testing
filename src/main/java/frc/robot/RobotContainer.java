// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ControlledGetToSpeed;
import frc.robot.commands.ControlledShoot;
import frc.robot.commands.DriverInvert;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.GetToSpeed;
import frc.robot.commands.Intake;
import frc.robot.commands.JiggleRobot;
import frc.robot.commands.KeepFromShooting;
import frc.robot.commands.LockSwerve;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.SwitchVisionState;
import frc.robot.commands.ToggleVision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Util;
 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public static final boolean atRecEvent = true;
  
  public static class Controllers {
    /**
     * Driver controller object. The driver controller is connected on port 0.
     * 
     * <p>Use to drive the swerve drive.
     */
    public final CommandXboxController driver = new CommandXboxController(0);

    /**
     * Operator controller object. The operator controller is connected on port 1.
     * 
     * <p>Use to control all robot functions that don't involve swerve.
     */
    public final CommandXboxController operator = new CommandXboxController(1);
  }

  public static class Subsystems {
    public SwerveSubsystem swerve;
    public Shooter shooter;
    public Climber climber;
    public Feeder feeder;
    public Vision vision;
  }

  public static class Commands {
    public ParallelCommandGroup standardDrive;
    public ParallelCommandGroup slowDrive;

    LockSwerve lockSwerve;

    public ClimbDown climbDown;
    public ClimbUp climbUp;
    public GetToSpeed getToSpeed;
    public AutoShoot autoShoot;
    public Intake intake;
    public ManualShoot manualShoot;
    public RotateToTarget rotateToTarget;
    public KeepFromShooting keepFromShooting;
    public JiggleRobot jiggleRobot;
    public Command autoShootIntoHub;

    public ControlledShoot lowPowerControlledShoot;
    public ControlledShoot meduimPowerControlledShoot;
    public ControlledShoot highPowerControlledShoot;
    public ControlledShoot maxPowerControlledShoot;

    public DriverInvert driverInvert;

    public SwitchVisionState switchVisionState;

    public ToggleVision toggleVision;
  }

  public final Controllers controllers = new Controllers();
  public final Subsystems subsystems = new Subsystems();
  public final Commands commands = new Commands();

  public final Settings settings = new Settings(this.controllers.driver, this.controllers.operator);

  public final PhotonCamera visionCamera = new PhotonCamera("Limelight"); 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize the logging module
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());
    // DataLogManager.stop();

    // Initialize subsystems
    this.subsystems.swerve = new SwerveSubsystem(this);
    this.subsystems.shooter = new Shooter(this);
    this.subsystems.climber = new Climber(this);
    this.subsystems.feeder = new Feeder(this);
    // this.subsystems.vision = new Vision(this);

    // Initialize commands
    this.commands.climbDown = new ClimbDown(this);
    this.commands.climbUp = new ClimbUp(this);
    this.commands.getToSpeed = new GetToSpeed(this);
    this.commands.autoShoot = new AutoShoot(this);
    this.commands.intake = new Intake(this);
    this.commands.manualShoot = new ManualShoot(this);
    this.commands.rotateToTarget = new RotateToTarget(this);
    this.commands.keepFromShooting = new KeepFromShooting(this);
    this.commands.jiggleRobot = new JiggleRobot(this);
    this.commands.autoShootIntoHub = new GetToSpeed(this).andThen(new AutoShoot(this));
    this.commands.lowPowerControlledShoot = new ControlledShoot(this, 0.55);
    this.commands.meduimPowerControlledShoot = new ControlledShoot(this, 0.70);
    this.commands.highPowerControlledShoot = new ControlledShoot(this, 0.85);
    this.commands.maxPowerControlledShoot = new ControlledShoot(this, 1);
    this.commands.driverInvert = new DriverInvert(this);
    this.commands.lockSwerve = new LockSwerve(this);

    this.commands.switchVisionState = new SwitchVisionState(this);

    this.commands.toggleVision = new ToggleVision(this);
    
    this.commands.standardDrive = new ParallelCommandGroup(this.subsystems.swerve.driveInputs(
      () -> (RobotContainer.atRecEvent ? 0.50 : 0.90) * -this.settings.driverSettings.getLeftY(),
      () -> (RobotContainer.atRecEvent ? 0.50 : 0.90) * -this.settings.driverSettings.getLeftX(),
      () -> (RobotContainer.atRecEvent ? 0.50 : 0.90) * -this.settings.driverSettings.getRightX()
    ));

    this.commands.slowDrive = new ParallelCommandGroup(this.subsystems.swerve.driveInputs(
      () -> (RobotContainer.atRecEvent ? 0.25 : 0.5) * -this.settings.driverSettings.getLeftY(),
      () -> (RobotContainer.atRecEvent ? 0.25 : 0.5) * -this.settings.driverSettings.getLeftX(),
      () -> (RobotContainer.atRecEvent ? 0.25 : 0.5) * -this.settings.driverSettings.getRightX()
    ));

    this.subsystems.swerve.setupPathPlanner();
    
    NamedCommands.registerCommand("rotateToTarget", this.commands.rotateToTarget);
    NamedCommands.registerCommand("getToSpeed", this.commands.getToSpeed);
    NamedCommands.registerCommand("intake", this.commands.intake);
    NamedCommands.registerCommand("autoShoot", this.commands.autoShoot);
    NamedCommands.registerCommand("climbup", this.commands.climbUp);
    NamedCommands.registerCommand("climbdown", this.commands.climbDown);
    NamedCommands.registerCommand("autoShootIntoHub", this.commands.autoShootIntoHub);
    NamedCommands.registerCommand("lowPowerControlledShoot", this.commands.lowPowerControlledShoot); //0.55
    NamedCommands.registerCommand("meduimPowerControlledShoot", this.commands.meduimPowerControlledShoot); //0.70
    NamedCommands.registerCommand("highPowerControlledShoot", this.commands.highPowerControlledShoot); //0.85
    NamedCommands.registerCommand("maxPowerControlledShoot", this.commands.maxPowerControlledShoot); // 1.0
    NamedCommands.registerCommand("switchVisionState", this.commands.switchVisionState);
    NamedCommands.registerCommand("sixtyPowerControlledShoot", new ControlledShoot(this, 0.6));
    NamedCommands.registerCommand("90PowerControlledShoot", new ControlledShoot(this, 0.90));
    NamedCommands.registerCommand("95PowerControlledShoot", new ControlledShoot(this, 0.95));
    NamedCommands.registerCommand("85PowerControlledShoot", new ControlledShoot(this, 0.85));
    NamedCommands.registerCommand("65PowerControlledShoot", new ControlledShoot(this, 0.65));
    NamedCommands.registerCommand("75PowerControlledShoot", new ControlledShoot(this, 0.75));
    NamedCommands.registerCommand("85PowerControlledGetToSpeed", new ControlledGetToSpeed(this, 0.85));
    NamedCommands.registerCommand("meduimPowerControlledGetToSpeed", new ControlledGetToSpeed(this, 0.70));
    NamedCommands.registerCommand("maxPowerControlledGetToSpeed", new ControlledGetToSpeed(this, 1.0));
 
    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);

    // Configure trigger bindings
    this.configureBindings();
  }

  /**
   * Robot periodic method. Runs at 50Hz (once every 20ms).
   */
  public void robotPeriodic() {
    SmartDashboard.putBoolean("inverted", this.settings.driverSettings.inverted);
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Bind driver buttons
    this.subsystems.swerve.setDefaultCommand(this.commands.standardDrive);
    this.settings.driverSettings.slowmode.whileTrue(this.commands.slowDrive);

    // Bind climber buttons
    this.settings.driverSettings.climbUpButton.whileTrue(this.commands.climbUp);
    this.settings.driverSettings.climbDownButton.whileTrue(this.commands.climbDown);

    // feeder button AND NOT shooter button -> intake
    // this.settings.operatorSettings.feederButton.and(this.settings.operatorSettings.shooterButton.negate()).whileTrue(this.commands.intake);
    
    // shooter button AND feeder button -> keep from shooting
    // this.settings.operatorSettings.shooterButton.and(this.settings.operatorSettings.feederButton).whileTrue(this.commands.keepFromShooting);

    // shooter button AND NOT feeder button -> manual shoot
    // this.settings.operatorSettings.shooterButton.and(this.settings.operatorSettings.feederButton.negate()).whileTrue(this.commands.manualShoot);

    //shooting for driver
    this.settings.driverSettings.shooterButton.whileTrue(this.commands.manualShoot);
    
    // auto rotate and shoot
    this.settings.driverSettings.autoRotateButton.whileTrue(this.commands.rotateToTarget);
    this.settings.operatorSettings.autoShootButton.whileTrue(this.commands.autoShoot);

    // controlled shoot bindings
    this.settings.operatorSettings.lowShootButton.whileTrue(this.commands.lowPowerControlledShoot);
    this.settings.operatorSettings.mediumShootButton.whileTrue(this.commands.meduimPowerControlledShoot);
    this.settings.operatorSettings.highShootButton.whileTrue(this.commands.highPowerControlledShoot);
    this.settings.operatorSettings.maxShootButton.whileTrue(this.commands.maxPowerControlledShoot);

    // jiggle Robot
    this.settings.driverSettings.jiggleRobotButton.whileTrue(this.commands.jiggleRobot);

    // invert controls
    this.settings.driverSettings.invertButton.onTrue(this.commands.driverInvert);
    
    // toggle vision
    this.settings.driverSettings.toggleVisionButton.onTrue(this.commands.toggleVision);

    //lock swerve
    this.settings.driverSettings.lockSwerveButton.whileTrue(this.commands.lockSwerve);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }

  public void setDriveMode() {
    this.configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    this.subsystems.swerve.setMotorBrake(brake);
  }
}
