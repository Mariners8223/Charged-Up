// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitiveV2.oriantion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class toggleOrienationMotors extends CommandBase {
  private static Orientation orientation;
  private boolean toggled;
  private double speed;
  /** Creates a new toggleOrienationMotors. */
  public toggleOrienationMotors(double speed) {
    orientation = Orientation.getInstance();
    toggled = false;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(toggled){
      toggled = false;
      orientation.stop();
    }
    else{
      toggled = true;
      orientation.setSpeed(speed);
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
