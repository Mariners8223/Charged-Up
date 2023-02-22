// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static Arm instance;
  private final ArmIOTalonSRX io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  

  private Arm(ArmIOTalonSRX io) {
    this.io = io;
  }

  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm(ArmIOTalonSRX.getInstance());
    return instance;
  }

  public boolean isAtRotationSetpoint() {
    return io.isArmAtSetpoint();
  }
  public boolean isAtExtensionSetpoint(){
    return io.isArmAtExtensionSetpoint();
  }

  public void stopRotationMotor() {
    io.stopRotation();
  }

  public void stopExtensionMotor() {
    io.stopExtension();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
  }

  public void extendToLengthMeters(double lengthMeters) {
    io.extendToLength(lengthMeters);
  }

  public void rotateToAngleDegrees(double desiredAngleDeg) {
    io.moveToAngle(desiredAngleDeg);
  }
}
