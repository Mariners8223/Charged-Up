// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivebase;

public class StartEnterRamp extends CommandBase {
  private static Drivebase chassis;
  /** Creates a new StartEnterRamp. */
  public StartEnterRamp() {
    chassis = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassis.drive(0, -1.5, 0);
    System.out.println("1");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roation = chassis.getRotationPID(chassis.getAngle());
    chassis.drive(0, -1.5, roation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(chassis.getRoll()) > 3;
  }
}
