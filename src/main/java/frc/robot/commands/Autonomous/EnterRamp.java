// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivebase;

public class EnterRamp extends CommandBase {
  private static Drivebase chassis;
  private double TimeSienceEnterdRamp;
  /** Creates a new EnterRamp. */
  public EnterRamp() {
    chassis = Drivebase.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TimeSienceEnterdRamp = Timer.getFPGATimestamp();
    chassis.drive(0, -2, 0);
    System.out.println("2");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double roation = chassis.getRotationPID(chassis.getAngle());
    chassis.drive(0, -2, roation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassis.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - TimeSienceEnterdRamp >= 1.8;
  }
}
