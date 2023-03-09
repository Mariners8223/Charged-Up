// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;
import frc.util.SequenceType;

public class setGripperState extends CommandBase {
  private static Gripper gripper;
  private static int count = 0;
  private SequenceType state;
  /** Creates a new setSolenoidState. */
  public setGripperState() {
    gripper = Gripper.getInstance();
    count++;
    count = count % 3;
    switch (count) {
      case 0:
        this.state = SequenceType.Cone;
        break;
      case 1:
        this.state = SequenceType.Cube;
      default:
        this.state = SequenceType.Off;
        break;
    }
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
