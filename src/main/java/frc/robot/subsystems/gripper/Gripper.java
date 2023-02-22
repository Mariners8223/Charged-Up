// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private static Gripper instance;

  private final GripperIO io;
  private final GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();

  private Gripper(GripperIO io) {
    this.io = io;
  }

  public static Gripper getInstance() {
    if (instance == null)
      instance = new Gripper(GripperIOTalonSRX.getInstance());
    return instance;
  }
  
  public boolean getGripperClosed() { return inputs.isClosed; }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Gripper", inputs);
  }

  public void setAngle(double Angle)
  {
    io.setAngle(Angle);
  }

  public boolean isAtSetpoint() {
    return io.isAtSetpoint();
  }

  /** Stops the gripper */
  public void stop() {
    io.setAngle(0);
  }
}
