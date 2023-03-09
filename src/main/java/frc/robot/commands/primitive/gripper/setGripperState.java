// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;
import frc.util.SequenceType;

public class setGripperState extends CommandBase {
  private static Gripper gripper;
  private static int count;
  private SequenceType state;
  /** Creates a new setSolenoidState. */
  public setGripperState(SequenceType state) {
    gripper = Gripper.getInstance();
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(state){
      case Cone:
        gripper.solenoidForward();
        break;
      
      case Cube:
        gripper.solenoidOff();
        break;
      
      case Off:
        gripper.solenoidBack();
        break;

      default:
        gripper.solenoidOff();
        break;

    }
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
