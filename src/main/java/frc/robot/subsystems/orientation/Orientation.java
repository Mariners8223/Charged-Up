// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.orientation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Orientation extends SubsystemBase {
  private static Orientation instance;
  private OrientationIOVictorSPX io;
  private OrientationIOInputsAutoLogged inputs = new OrientationIOInputsAutoLogged();
  
  private Orientation(OrientationIOVictorSPX io) {
    this.io = io;
  }

  public static Orientation getInstance() {
    if (instance == null)
      instance = new Orientation(OrientationIOVictorSPX.getInstance());
    return instance;
  }

  public void setSpeed(double speed) {
    io.isRunning = true;
    io.setPercent(speed);
  }

  public void stop() {
    io.isRunning = false;
    io.setPercent(0);
  }

  public void toggleSolenoid() {
    io.toggleOrientation();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
