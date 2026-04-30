package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * <a href="https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf"> LED Colors
 */
public class LEDs extends SubsystemBase {
  private final SendableChooser<LEDs.Color> colorChooser = new SendableChooser<>();

  private final SwerveSubsystem swerveSubsystem;

  private Spark LEDs = new Spark(Constants.LED_PORT);
  private Color color = Color.PURPLE;
  @SuppressWarnings("unused")
  private Shooter shooter; // TODO - implement shooter object

  public LEDs(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;

    for (Color color : Color.values()) {
      colorChooser.addOption(color.name(), color);
    }
    colorChooser.setDefaultOption("PURPLE", Color.PURPLE);
    SmartDashboard.putData("Color Chooser", colorChooser);
    //?
    // LEDs = new Spark(Constants.LED_PORT);
  }

  @Override
  public void periodic() {

    if (DriverStation.isDisabled()) {
      setColor(Color.PURPLE);
    }

    else if (this.swerveSubsystem.isLinedUp()) {
      setColor(Color.GREEN);
    }

    /*
    else if (shooter.isHolding()) {
    setColor(color.YELLOW);
    }
    */
    else {
      setColor(Color.ORANGE);
    }

    // LEDs.set(color.getValue());
    // LEDs.set(colorChooser.getSelected().colorToSpeed());
    LEDs.set(this.color.colorToSpeed());
  }

  public void setColor(Color color) {
  this.color = color;
  }

  public static enum Color {
    PURPLE (0.91),
    HEARTBEAT_RED (-0.25),
    HEARTBEAT_BLUE (-0.23),
    YELLOW (0.69),
    FIRE_LARGE(-0.57),
    CONFETTI(-0.87),
    CHASE_RED(-0.31),
    CHASE_BLUE(-0.29),
    BLACK(0.99),
    RED_STROBE(-0.11),
    OCEAN(-0.95),
    LAVA(-0.93),
    RED(0.61),
    BLUE(0.87),
    FOREST(-0.91),
    ORANGE(0.65),
    GREEN(0.77);

    private final double value;

    Color(double value) {
      this.value = value;
    }

    public double colorToSpeed() {
      return value;
    }
  }

  public void useChooser() {
    setColor(colorChooser.getSelected());
  }
}