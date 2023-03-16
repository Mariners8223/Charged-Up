// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivebase;

public class BalanceOnRamp extends CommandBase {
  private static Drivebase drivebase;
  private double timeSeinceLastAdjust;
  private double cooldown;
  /** Creates a new BalanceOnRamp. */
  public BalanceOnRamp() {
    drivebase = Drivebase.getInstance(); //you know the dril
    timeSeinceLastAdjust = 0;
    cooldown = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase); //you know the drill
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cooldown = Timer.getFPGATimestamp();
    System.out.println("3");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speedDircation = -1;
    if(drivebase.getRoll() > 0){
      speedDircation = 1;
    }


    double speedMultiplyer = 0;
    double pitch = Math.abs(drivebase.getRoll());
    
    if(Timer.getFPGATimestamp() - cooldown < 0.5){
      if(pitch > 10){
        speedMultiplyer = 1;
        timeSeinceLastAdjust = Timer.getFPGATimestamp();
      }
      else if(pitch > 5){
        speedMultiplyer = 0.7;
        timeSeinceLastAdjust = Timer.getFPGATimestamp();
      }
    }
    else if(Timer.getFPGATimestamp() - cooldown > 1.3){
      cooldown = Timer.getFPGATimestamp();
    }

    double roation = drivebase.getRotationPID(drivebase.getAngle());
    drivebase.drive(0, -(speedDircation * speedMultiplyer), roation);

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0);
    //took 2 secs
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - timeSeinceLastAdjust >= 2);
  }
}
