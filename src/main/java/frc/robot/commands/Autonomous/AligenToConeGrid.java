// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.drivetrain.Drivebase;

public class AligenToConeGrid extends CommandBase {
  private static Drivebase drivebase;
  private static LimeLight limeLight;
  private PIDController controller;
  /** Creates a new AligenToConeGrid. */
  public AligenToConeGrid() {
    drivebase = Drivebase.getInstance();
    limeLight = LimeLight.getInstance();
    controller = new PIDController(0.2, 0, 0);
    controller.setSetpoint(0);
    controller.setTolerance(5);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase, limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = limeLight.getPitchToTarget();
    pitch = MathUtil.clamp(controller.calculate(pitch), -2.5, 2.5);
    drivebase.drive(0, pitch, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebase.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
