// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripperV2;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperIOSolonoid extends SubsystemBase {
  private static GripperIOSolonoid instance;
  private static DoubleSolenoid coneSolonoid;
  private static DoubleSolenoid cubeSolonoid;
  /** Creates a new GripperIOSolonoid. */
  private GripperIOSolonoid() {
    coneSolonoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    cubeSolonoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 11);
  }
  public static GripperIOSolonoid getInstance(){
    if(instance == null){
      instance = new GripperIOSolonoid();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
