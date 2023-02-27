// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripperV2;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class GripperV2 extends SubsystemBase {
  private static GripperV2 instance;
  private static DoubleSolenoid solenoid;
  /** Creates a new Gripper. */
  private GripperV2() {
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, RobotConstants.GRIPPER_DOUBLE_SOLENOID_PORTS[0], RobotConstants.GRIPPER_DOUBLE_SOLENOID_PORTS[1]);
  }
  public static GripperV2 getInstance(){
    if(instance == null){
      instance = new GripperV2();
    }
    return instance;
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
    // This method will be called once per scheduler run
  }
}
