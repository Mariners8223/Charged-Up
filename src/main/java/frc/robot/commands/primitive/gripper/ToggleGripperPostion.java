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
  private enum toggle{
    open, closed, free
  }
  toggle toggleEnum = toggle.closed;
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
        switch(toggleEnum){
          default:
            break;
          
          case free:
            toggleEnum = toggleEnum.open;
            gripper.solenoidBack();
            break;

          case open:
            toggleEnum = toggleEnum.free;
            gripper.solenoidOff();
            break;

          case closed:
            toggleEnum = toggleEnum.free;
            gripper.solenoidForward();
            break;
        }
        break;

      case Cone:
        switch(toggleEnum){
          default:
            break;

          case free:
            toggleEnum = toggleEnum.closed;
            gripper.solenoidForward();
            break;

          case open:
            toggleEnum = toggleEnum.closed;
            gripper.solenoidForward();
            break;

          case closed:
            toggleEnum = toggleEnum.open;
            gripper.solenoidBack();
            break;
        }
        break;
    }
  }
}
