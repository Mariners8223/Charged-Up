// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripperV2.GripperV2;

public class toggleGripperV2Solenoid extends CommandBase {
  private static GripperV2 gripperV2;
  private int shit;
  /** Creates a new toggleGripperV2Solenoid. */
  public toggleGripperV2Solenoid(int shit) {
    gripperV2 = GripperV2.getInstance();
    this.shit = shit;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(gripperV2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(shit == 1){
      gripperV2.solenoidForward();
      System.out.println(1);
    }
    else if(shit == 2){
      gripperV2.solenoidBack();
      System.out.println(2);
    }
    else if(shit == 3){
      gripperV2.solenoidOff();
      System.out.println(3);
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
    return false;
  }
}