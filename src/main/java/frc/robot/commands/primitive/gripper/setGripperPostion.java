// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;

import org.opencv.aruco.GridBoard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.gripper.Gripper;
import frc.util.SequenceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setGripperPostion extends InstantCommand {
  private static Gripper gripper;
  private SequenceType state;
  public setGripperPostion(SequenceType state) {
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
}
