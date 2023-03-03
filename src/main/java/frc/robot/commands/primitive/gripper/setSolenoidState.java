// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.gripper;

import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripper.Gripper;
import frc.util.SequenceType;

public class setSolenoidState extends CommandBase {
  private static Gripper gripper;
  private SequenceType state;
  /** Creates a new setSolenoidState. */
  public setSolenoidState(SequenceType state) {
    gripper = gripper.getInstance();
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

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
