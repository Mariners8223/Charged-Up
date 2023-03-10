// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivebase;

public class BalanceOnRamp extends CommandBase {
  private static Drivebase drivebase;
  private boolean MinorAdjust;
  private double timeSeinceLastAdjust;
  /** Creates a new BalanceOnRamp. */
  public BalanceOnRamp() {
    drivebase = Drivebase.getInstance(); //you know the dril
    MinorAdjust = false; //makes the command start with not adjust
    timeSeinceLastAdjust = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase); //you know the drill
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedDircation = 1;
    if(drivebase.getPitch() > 0){
      speedDircation = -1;
    }


    double speedMultiplyer = 0;

    if(Math.abs(drivebase.getPitch())  > 10){
      speedMultiplyer = 0.5;
      timeSeinceLastAdjust = Timer.getFPGATimestamp();
    }
    else if(Math.abs(drivebase.getPitch()) > 3){
      speedMultiplyer = 0.3;
      timeSeinceLastAdjust = Timer.getFPGATimestamp();
    }

    drivebase.drive(speedDircation * speedMultiplyer, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - timeSeinceLastAdjust >= 3);
  }
}
