package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivebase;

public class Drive extends CommandBase {
  private Drivebase swerve;
  private boolean fieldOriented;
  private double xSpeed;
  private double ySpeed;
  private double rotation;
  public Drive(boolean isFieldOriented) {
    this.fieldOriented = isFieldOriented;
    swerve = Drivebase.getInstance();
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xSpeed = RobotContainer.getRawAxis(0);
    ySpeed = -RobotContainer.getRawAxis(1);
    rotation = RobotContainer.getRawAxis(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = RobotContainer.getRawAxis(0);
    ySpeed = -RobotContainer.getRawAxis(1);
    rotation = RobotContainer.getRawAxis(4);
    swerve.Drive(xSpeed, ySpeed, rotation, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.Drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
