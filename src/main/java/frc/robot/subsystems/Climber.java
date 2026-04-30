package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@Logged
public class Climber extends SubsystemBase {
  public final TalonFX motor = new TalonFX(Constants.ID.CLIMBER_ID);
  CommandXboxController operatorController;
  DigitalInput climbUpLimitSwitchInput = new DigitalInput(Constants.ID.LIMIT_SWITCH_ID);

  private ArrayList<Boolean> limitSwitchIDs = new ArrayList<>();
  private ArrayList<DigitalInput> digitalInputs = new ArrayList<>();

  private boolean previousState = false;

  public Climber(RobotContainer robotContainer) {
    operatorController = robotContainer.controllers.operator;

    // for (int i = 0 ; i < 31 ; i++) {
    //   digitalInputs.add(new DigitalInput(i));
    //   limitSwitchIDs.add(false);
    // }
  }

  // Put into variable then log/return current state (triggered or untriggered)
  public boolean ClimbUpDone() {
    previousState = climbUpLimitSwitchInput.get();

    if (previousState) {
      motor.setPosition(0);
    }
    
    SmartDashboard.putBoolean("Triggered", previousState);
    return previousState;

    // return false;
  }

  public void printInputStates() {
    for (int i = 0 ; i < 31 ; i++) {
      limitSwitchIDs.set(i, digitalInputs.get(i).get());
    }

    // print trues

    System.out.print("IDS SET TO TRUE: ");
    for (int i = 0 ; i < 31 ; i++) {
      System.out.print((limitSwitchIDs.get(i) ? i : "") + ", ");
    }
    
    System.out.println();

    // print falses

    System.out.print("IDS SET TO FALSE: ");
    for (int i = 0 ; i < 31 ; i++) {
      System.out.print((limitSwitchIDs.get(i) ? "" : i) + ", ");
    }

    System.out.println();
  }
}
