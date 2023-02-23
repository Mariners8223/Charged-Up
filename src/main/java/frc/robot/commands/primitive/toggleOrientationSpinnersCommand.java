// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

public class toggleOrientationSpinnersCommand extends CommandBase {
  private Orientation orientation;
  private SequenceType sqncType;

  public toggleOrientationSpinnersCommand(SequenceType type) {
    this.orientation = Orientation.getInstance();
    this.sqncType = type;

    addRequirements(orientation);
  }

  @Override
  public void initialize() {
    if(!orientation.getIsRunning()){
      orientation.toggleSolenoid();
      if(sqncType == SequenceType.Cube)
        orientation.setSpeed(0.4);
      else
        orientation.setSpeed(0.6);
    }
    else
      orientation.stop();
      orientation.toggleSolenoid();
    
  }
  @Override
  public void end(boolean interrupted) {
    orientation.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
