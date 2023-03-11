// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.orientation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetRampSolenoidState extends InstantCommand {
  private static Orientation orientation;
  private SequenceType sequenceType;
  public SetRampSolenoidState(SequenceType sequenceType) {
    orientation = Orientation.getInstance();
    this.sequenceType = sequenceType;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(sequenceType){
      case Raised:
        orientation.raiseRamp();
        break;
      
      case Lowered:
        orientation.lowerRamp();
        break;
      
        default:
          orientation.toggleRampSolenoid();
          break;
    }
  }
}
