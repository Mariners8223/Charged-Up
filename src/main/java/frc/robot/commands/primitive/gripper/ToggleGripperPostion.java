// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.gripper.Gripper;
import frc.util.SequenceType;
import frc.util.ToggleSwitch;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleGripperPostion extends InstantCommand {
  private ToggleSwitch toggleSwitch;
  private Gripper gripper;
  private SequenceType state;
  public ToggleGripperPostion(SequenceType state) {
    toggleSwitch = RobotContainer.toggleSwitch;
    gripper = Gripper.getInstance();
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(state){
      default:
        break;

      case Cube:
        toggleSwitch.setSwitchState(2, false);
        if(toggleSwitch.ToggleTheSwitchAndGetToggleSwitchValue(1)){
          gripper.solenoidOff();
        }
        else{
          gripper.solenoidBack();
        }
        break;

      case Cone:
        toggleSwitch.setSwitchState(1, false);
        if(toggleSwitch.ToggleTheSwitchAndGetToggleSwitchValue(2)){
          gripper.solenoidForward();
        }
        else{
          gripper.solenoidBack();
        }
        break;
    }
  }
}
