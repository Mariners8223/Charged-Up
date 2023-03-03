// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.oriantion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

public class SetOrienationSolenoidSate extends CommandBase {
  private static Orientation orientation;
  private SequenceType solenoid;
  private SequenceType state;
  /** Creates a new SetOrienationSolenoidSate. */
  public SetOrienationSolenoidSate(SequenceType solenoid, SequenceType state) {
    orientation = Orientation.getInstance();
    this.solenoid = solenoid;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(solenoid){
        case Ramp:
          switch(state){
              case On:
                orientation.SetRampSolenoidState(true);
                break;
              
              case Off:
                orientation.SetRampSolenoidState(false);
                break;
          }
        break;

        case Orienation:
          switch(state){
              case On:
                orientation.SetUpSolenoid(true);
                break;
              
              case Off:
                orientation.SetUpSolenoid(false);
                break;
          }
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
