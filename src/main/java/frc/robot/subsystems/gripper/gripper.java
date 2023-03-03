// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private static Gripper instance;
  private GripperIO io;

  private Gripper(GripperIO io) {
    this.io = io;
  }

  public static Gripper getInstance() {
    if(instance == null)
      instance = new Gripper(GripperIOSolenoid.getInstance());
    return instance;
  }

  public void toggleSolenoid() {
    io.toggleSolenoid();
  }

  public boolean isClosed() {
    return io.isClosed();
  }

  public void solenoidForward(){
    io.solenoidForward();
  }

  public void solenoidBack(){
    io.solenoidBack();
  }
  
  public void solenoidOff(){
    io.solenoidOff();
  }

  @Override
  public void periodic() {
  }
}
