// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setSpeedForm extends InstantCommand {
  public setSpeedForm() {

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond == 2){
      Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond = 4;
    }
    else{
      Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond = 2;
    }
  }
}
