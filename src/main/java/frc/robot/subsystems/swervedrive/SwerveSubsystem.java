// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Vector2;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification.NotificationLevel;
import frc.robot.util.Util;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.epilogue.Logged;

@Logged 
public class SwerveSubsystem extends SubsystemBase {
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private Vision vision;

  /**
   * Swerve drive object.
   */
  private final SwerveDrive swerveDrive;

  private final DoubleLogEntry xPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "xPosition");
  private final BooleanLogEntry teleopLog = new BooleanLogEntry(DataLogManager.getLog(), "Teleop");
  private final BooleanLogEntry autoLog = new BooleanLogEntry(DataLogManager.getLog(), "Autonomous");
  private final DoubleLogEntry rotationLog = new DoubleLogEntry(DataLogManager.getLog(), "Rotation");

  /**
   * AprilTag field layout.
   */
  public final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /**
   * Enable vision odometry updates while driving.
   */
  public final boolean visionDriveTest = true;

  public boolean useVision = false;
  public boolean isSwitchedToTelop = false;

  public boolean allowVisionPose = true;
  public boolean climbing = false;
  
  public final PathConstraints pathConstraints;

  private final RobotContainer robotContainer;

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  
  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  @SuppressWarnings("deprecation")
  public SwerveSubsystem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      this.swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED, Constants.STARTING_POSE);

      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (IOException exception) {
      throw new RuntimeException(exception);
    }

    // Heading correction should only be used while controlling the robot via angle.
    this.swerveDrive.setHeadingCorrection(!Robot.isSimulation());
    
    // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    this.swerveDrive.setCosineCompensator(!Robot.isSimulation());
    
    this.swerveDrive.chassisVelocityCorrection = true;

    // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    this.swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

    // Enable if you want to resynchronize your absolute encoders and motor encoders  when they are not moving.
    this.swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    this.swerveDrive.pushOffsetsToEncoders();

    if (this.visionDriveTest) {
      this.setupPhotonVision();

      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      this.swerveDrive.stopOdometryThread();
    }

    final double maxLinearVelocity = swerveDrive.getMaximumChassisVelocity();
    final double maxAngularVelocity = swerveDrive.getMaximumChassisAngularVelocity();
    final double maxAngularAcceleration = Units.degreesToRadians(2131);
    pathConstraints = new PathConstraints(maxLinearVelocity, 4.0, maxAngularVelocity, maxAngularAcceleration);
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision() {
    this.vision = new Vision(swerveDrive::getPose, swerveDrive.field, this.robotContainer);
  }

  public Vision getVision() {
    return this.vision;
  }

  @Override
  public void periodic() {
    if (!isSwitchedToTelop && DriverStation.isTeleop()) {
      isSwitchedToTelop = true;
      useVision = true;
    }

    final Pose2d robotPose = this.getPose();
    final Rotation2d robotYaw = this.getYaw();

    this.teleopLog.append(DriverStation.isTeleopEnabled());
    this.autoLog.append(DriverStation.isAutonomousEnabled());
    this.xPositionLog.append(robotPose.getX());
    this.rotationLog.append(robotYaw.getDegrees());

    // final Optional<Pose2d> cameraPose = vision.getFieldPose();
    // if (cameraPose.isPresent()) {
    //   this.swerveDrive.addVisionMeasurement(cameraPose.get(), Timer.getFPGATimestamp());
    //   System.out.println(cameraPose.toString());
    // }
    
    // vision.updatePoseEstimation(swerveDrive);
    SmartDashboard.putNumber("Y velocity", this.getRobotVelocity().vyMetersPerSecond);
    SmartDashboard.putNumber("X velocity", this.getRobotVelocity().vxMetersPerSecond);
    SmartDashboard.putNumber("Angle velocity", this.getRobotVelocity().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Gyro Value",Math.toDegrees(this.swerveDrive.getGyro().getRawRotation3d().getZ()));

    // final Optional<Transform3d> tagPose = vision.getTagPose(4);
    // if (tagPose.isPresent()) {
    //   final double targetYaw = tagPose.get().getRotation().getZ();
    //   SmartDashboard.putNumber("robot angle", Math.toDegrees(this.getYaw().getRadians()));
    //   SmartDashboard.putNumber("target angle", Math.toDegrees(targetYaw));
    // }

    // if (cameraPose.isPresent()) {
    //   final Pose2d estimatedPose = cameraPose.get();
    //   SmartDashboard.putNumber("Estimated Robot Pose X", estimatedPose.getX());
    //   SmartDashboard.putNumber("Estimated Robot Pose Y", estimatedPose.getY());
    //   SmartDashboard.putNumber("Estimated Robot Angle", estimatedPose.getRotation().getDegrees());
    // }
    
    SmartDashboard.putNumber("Robot Field X", robotPose.getX());
    SmartDashboard.putNumber("Robot Field Y", robotPose.getY());
    SmartDashboard.putNumber("Robot Field Angle", robotPose.getRotation().getDegrees());
    
    final ChassisSpeeds speed = this.getRobotVelocity();
    SmartDashboard.putNumber("X Velocity", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("Y Velocity", speed.vyMetersPerSecond);
    SmartDashboard.putNumber("Rotation Velocity", speed.omegaRadiansPerSecond);

    // // not moving
    // final boolean isStationaryX = MathUtil.isNear(0, speed.vxMetersPerSecond, 0.0001);
    // final boolean isStationaryY = MathUtil.isNear(0, speed.vyMetersPerSecond, 0.0001);
    // final boolean isNotRotating = MathUtil.isNear(0, speed.omegaRadiansPerSecond, 0.0001);
    // if (isStationaryX && isStationaryY && isNotRotating) {
    //   final Optional<Pose2d> visionPose = vision.getFieldPose();
    //   if (visionPose.isPresent()) {
    //     System.out.println("update odometry");

    //     this.resetOdometry(visionPose.get());
    //   }
    // }
    if (visionDriveTest) {
      swerveDrive.updateOdometry();
      
      if (DriverStation.isTeleop() && !vision.isDisabled()) {
        vision.updatePoseEstimation(swerveDrive);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // vision.updateSimulation(this);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    try {
      final RobotConfig config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = true;
      
      AutoBuilder.configure(
        this::getPose,          // Robot pose supplier
        this::resetOdometry,    // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

        (speedsRobotRelative, moduleFeedForwards) -> {
          if (enableFeedforward) {
            final SwerveModuleState states[] = this.swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative);
            final Force[] feedforwardForces = moduleFeedForwards.linearForces();

            this.swerveDrive.drive(speedsRobotRelative, states, feedforwardForces);
          } else {
            this.swerveDrive.setChassisSpeeds(speedsRobotRelative);
          }
        },

        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController(
            // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5.0, 0.25, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
        ),

        config,              // The robot configuration
        this::isRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
        this                 // Reference to this subsystem to set requirements
      );

      final LocalADStar localADStar = new LocalADStar();
      Pathfinding.setPathfinder(localADStar);
    } catch (Exception exception) {
      exception.printStackTrace();
    }
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @param camera {@link PhotonCamera} to communicate with.
   * @return A {@link Command} which will run the alignment.
   */
  @SuppressWarnings("removal")
  public Command aimAtTarget(PhotonCamera camera) {
    return run(() -> {
      // Get the target rotation form the latest photon pipeline result

      final PhotonPipelineResult latestResult = camera.getLatestResult();
      final Rotation2d rotation = Rotation2d.fromDegrees(latestResult.getBestTarget().getYaw());

      // Drive the robot

      final ChassisSpeeds velocity = this.getTargetSpeeds(0, 0, rotation);
      this.drive(velocity);
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command driveInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return super.run(() -> {
      // Make the robot move
      double x = translationX.getAsDouble() * this.swerveDrive.getMaximumChassisVelocity();
      double y = translationY.getAsDouble() * this.swerveDrive.getMaximumChassisVelocity();
      double angle = angularRotationX.getAsDouble() * this.swerveDrive.getMaximumChassisAngularVelocity();

      this.swerveDrive.drive(new Translation2d(x, y), angle, true, true);
    });
  }

  public Command driveAndSpin() {
    return super.run(()-> {
      final ChassisSpeeds velocity = new ChassisSpeeds(0.5, 0, Math.PI);
      this.driveFieldOriented(velocity);
    });
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while
    final double maxLinearVelocity = swerveDrive.getMaximumChassisVelocity();
    final double maxAngularVelocity = swerveDrive.getMaximumChassisAngularVelocity();
    final double maxAngularAcceleration = Units.degreesToRadians(2131);
    final PathConstraints constraints = new PathConstraints(maxLinearVelocity, 4.0, maxAngularVelocity, maxAngularAcceleration);

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0)).andThen(()-> {
      final Elastic.Notification notification = new Elastic.Notification(NotificationLevel.INFO, "", "Drove to " + pose);
      Elastic.sendNotification(notification);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return super.run(() -> {
      final double xSpeed = translationX.getAsDouble();
      final double ySpeed = translationY.getAsDouble();

      final Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(xSpeed, ySpeed), 0.8);
      
      SmartDashboard.putNumber("Controller Angle", scaledInputs.getAngle().getDegrees());
      SmartDashboard.putNumber("Controller X Speed", scaledInputs.getX());
      SmartDashboard.putNumber("Controller Y Speed", scaledInputs.getY());
      
      // Make the robot move
      final ChassisSpeeds speeds = this.swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), this.swerveDrive.getOdometryHeading().getRadians(), this.swerveDrive.getMaximumChassisVelocity());

      if (MathUtil.applyDeadband(headingX.getAsDouble(), 0.1) == 0 && MathUtil.applyDeadband(headingY.getAsDouble(), 0.1) == 0) {
        speeds.omegaRadiansPerSecond = 0;
      }
      
      SmartDashboard.putNumber("Robot Movement Speeds X", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Robot Movement Speeds Y", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Robot Movement Speeds Angle", speeds.omegaRadiansPerSecond);

      this.driveFieldOriented(speeds);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation Rotation as a value between [-1, 1] converted to radians.
   * @return Drive command.
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return super.run(() -> {
      final double xSpeed = translationX.getAsDouble();
      final double ySpeed = translationY.getAsDouble();

      // Make the robot move
      this.driveFieldOriented(this.swerveDrive.swerveController.getTargetSpeeds(xSpeed, ySpeed, rotation.getAsDouble() * Math.PI, this.swerveDrive.getOdometryHeading().getRadians(), this.swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand() {
    return Commands.none();
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand() {
    final SysIdRoutine sysIdRoutine = SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, this.swerveDrive);

    return SwerveDriveTest.generateSysIdCommand(sysIdRoutine, 3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    System.out.println("centered");
    return run(() -> Arrays.asList(this.swerveDrive.getModules()).forEach(swerveModule -> swerveModule.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return Commands.deferredProxy(() -> Commands.run(() -> {
      this.drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0));

      System.out.println(this.swerveDrive.getPose().getTranslation());
    }, this).until(() -> this.swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters));
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(kS, kV, kA); 
    this.swerveDrive.replaceSwerveModuleFeedforward(driveFeedForward);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return super.run(() -> {
      final double xSpeed = translationX.getAsDouble() * this.swerveDrive.getMaximumChassisVelocity();
      final double ySpeed = translationY.getAsDouble() * this.swerveDrive.getMaximumChassisVelocity();

      // Make the robot move
      final Translation2d linearVelocity = SwerveMath.scaleTranslation(new Translation2d(xSpeed, ySpeed), 0.8);                    // translation * 0.8
      final double angularVelocity = Math.pow(angularRotationX.getAsDouble(), 3) * this.swerveDrive.getMaximumChassisAngularVelocity(); // (rotation ^ 3) * chassis angular velocity
      this.swerveDrive.drive(linearVelocity, angularVelocity, true, false);
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field-relative and 
   * robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot,
   * in meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   * positive y is torwards port (left).  In field-relative mode, positive x is away from the
   * alliance wall (field North) and positive y is torwards the left wall when looking through the
   * driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by 
   * field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    // Open loop is disabled since it shouldn't be used most of the time.
    this.swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    this.swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    this.swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return this.swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset
   * when calling this method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    this.swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    
    return this.swerveDrive.getPose();
    
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    this.swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    final Pose2d pose = this.getPose();
    this.resetOdometry(new Pose2d(pose.getX(), pose.getY(), new Rotation2d()));
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    final Optional<Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (this.isRedAlliance()) {
      this.zeroGyro();

      // Set the pose 180 degrees
      final Pose2d initialHolonomicPose = new Pose2d(this.getPose().getTranslation(), Rotation2d.fromDegrees(180));
      this.resetOdometry(initialHolonomicPose);
    } else {
      this.zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    this.swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return this.getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which
   * direction. The other for the angle of the robot.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return this.getTargetSpeeds(xInput, yInput, headingX, headingY, Units.feetToMeters(1));
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY, double maxSpeed) {
    final Translation2d inputTranslation = new Translation2d(xInput, yInput);
    final Translation2d scaledInputs = SwerveMath.cubeTranslation(inputTranslation);
    final ChassisSpeeds speeds = this.swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX, headingY, this.getHeading().getRadians(), Constants.MAX_SPEED);
    return speeds;
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    final Translation2d inputTranslation = new Translation2d(xInput, yInput);
    final Translation2d scaledInputs = SwerveMath.cubeTranslation(inputTranslation);

    return this.swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), angle.getRadians(), this.getHeading().getRadians(), Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return this.swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return this.swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return this.swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return this.swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    this.swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return this.swerveDrive.getPitch();
  }

  public Rotation2d getYaw() {
    return this.swerveDrive.getOdometryHeading();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    final Pose2d robotPose = new Pose2d(3, 3, Rotation2d.fromDegrees(65));
    this.swerveDrive.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());
  }

  public boolean isLinedUp() {
    Vector2 targetPosition = Util.getTargetPosition();
    double robotX = getPose().getX();
    double robotY = getPose().getY();

    double targetAngle = Math.atan2(targetPosition.y-robotY, targetPosition.x-robotX);
    double robotAngle = getYaw().getRadians();

    return Math.abs(robotAngle - targetAngle) < 0.1;
  }
}