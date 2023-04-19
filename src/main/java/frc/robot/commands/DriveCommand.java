package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.Drivebase;
import frc.util.humanIO.CommandPS5Controller;

public class DriveCommand extends CommandBase {
  Drivebase swerve;
  private double xSpeed;
  private double ySpeed;
  private double rotation;
  CommandPS5Controller driveController;
  public DriveCommand() {
    swerve = Drivebase.getInstance();
    driveController = RobotContainer.getDriveController();
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    rotation = 0;
    xSpeed = 0;
    ySpeed = 0;
  }

  // public void execute() {
  //   xSpeed = RobotContainer.calculateDeadband(driveController.getRawAxis(0))
  //       * SwerveModuleConstants.freeSpeedMetersPerSecond;
  //   ySpeed = -RobotContainer.calculateDeadband(driveController.getRawAxis(1))
  //       * SwerveModuleConstants.freeSpeedMetersPerSecond;
  //   if (RobotContainer.calculateDeadband(driveController.getRawAxis(2)) != 0) {
  //     rotation = RobotContainer.calculateDeadband(driveController.getRawAxis(2)) * 10;
  //   } else rotation = swerve.getRotationPID(swerve.getAngle());
  //   swerve.drive(xSpeed ,ySpeed , rotation);
  // }
  @Override
  public void execute() {
    ySpeed = RobotContainer.calculateDeadband(driveController.getRawAxis(0))
      * SwerveModuleConstants.freeSpeedMetersPerSecond;
    xSpeed = -RobotContainer.calculateDeadband(driveController.getRawAxis(1))
      * SwerveModuleConstants.freeSpeedMetersPerSecond;
    if (RobotContainer.calculateDeadband(driveController.getRawAxis(2)) != 0) {
      rotation = RobotContainer.calculateDeadband(driveController.getRawAxis(2)) * 10;
      swerve.drive(ySpeed, xSpeed, rotation, Rotation2d.fromDegrees(swerve.getAngleDegrees()));
    } else {
      rotation = Math.abs((swerve.calculateError() - swerve.getDesiredAngle().getDegrees())) <= 2 ? 0 : swerve.calculateReference(Rotation2d.fromDegrees(swerve.getAngleDegrees()));
      swerve.drive(ySpeed, xSpeed, rotation, swerve.getDesiredAngle());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}