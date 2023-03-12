// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.primitive.orientation.intakeCommand;

public class LimeLight extends SubsystemBase {
  private static LimeLight instance;
  /** Creates a new LimeLight. */
  private LimeLight() {}

  public static LimeLight getInstance(){
    if(instance == null){
      instance = new LimeLight();
    }
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
