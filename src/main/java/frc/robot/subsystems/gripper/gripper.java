// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.Constants.RobotConstants;

public class Gripper extends SubsystemBase {
  private static Gripper instance;
  private static DoubleSolenoid solenoid;
  /** Creates a new Gripper. */
  private Gripper() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);
    solenoid.set(Value.kForward);
  }
  public static Gripper getInstance(){
    if(instance == null){
      instance = new Gripper();
    }
    return instance;
  }

  public void toggleSolenoid(){
    solenoid.toggle();
  }

  public boolean isClosed() {
    return solenoid.get() == Value.kForward;
  }

  public void solenoidForward(){
    solenoid.set(Value.kForward);
  }

  public void solenoidBack(){
    solenoid.set(Value.kReverse);
  }
  
  public void solenoidOff(){
    solenoid.set(Value.kOff);
  }
  @Override
  public void periodic() {
  }
}
