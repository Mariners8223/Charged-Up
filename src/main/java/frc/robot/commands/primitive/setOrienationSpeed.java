// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.orientation.Orientation;

public class setOrienationSpeed extends CommandBase {
  private static Orientation orientation;
  private static double speed;
  private static boolean state;
  /** Creates a new setOrienationSpeed. */
  public setOrienationSpeed(double speed, boolean state) {
    orientation = Orientation.getInstance();
    this.speed = speed;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(state){orientation.setSpeed(speed);}
    else{orientation.stop();}
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
