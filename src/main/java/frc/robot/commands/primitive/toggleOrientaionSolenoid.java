// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.orientation.Orientation;

public class toggleOrientaionSolenoid extends CommandBase {
  private static Orientation orientation;
  private boolean mode;
  private boolean solenoid;
  /** Creates a new orienation. */
  public toggleOrientaionSolenoid(boolean solenoid, boolean mode) {
    orientation = Orientation.getInstance();
    this.mode = mode;
    this.solenoid = solenoid;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(orientation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(solenoid){ orientation.SetRampSolenoidState(mode);}
    else{ orientation.SetUpSolenoid(mode);}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
