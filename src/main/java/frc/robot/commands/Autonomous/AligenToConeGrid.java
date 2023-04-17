// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.Drivebase;

public class AligenToConeGrid extends CommandBase {
  private PIDController controller;
  private Drivebase swerve;
  private LimeLight vision;
  private double distanceX;
  /** Creates a new AligenToConeGrid. */
  public AligenToConeGrid() {
    vision = LimeLight.getInstance();
    swerve = Drivebase.getInstance();
    controller = new PIDController(0.2, 0, 0);
    controller.setSetpoint(0);
    controller.setTolerance(5);
    distanceX = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distanceX = vision.getDistanceX();
    double speed = controller.calculate(distanceX);
    double roataion = swerve.getRotationPID(swerve.getAngle());
    swerve.drive(0, speed, roataion);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
