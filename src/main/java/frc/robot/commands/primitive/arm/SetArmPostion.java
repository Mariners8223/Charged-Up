// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArmPostion extends InstantCommand {
  private boolean state;
  public SetArmPostion(boolean state) {
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state){
      RobotContainer.setArmPostion(RobotContainer.getArmPosition() + 1);
    }
    else{
      RobotContainer.setArmPostion(RobotContainer.getArmPosition() - 1);
    }

    if(RobotContainer.getArmPosition() < 0){
      RobotContainer.setArmPostion(0);
    }
  }
}
