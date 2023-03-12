// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.primitive.Wait;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.calibrateArm;
import frc.robot.commands.primitive.arm.extendArmToLength;
import frc.robot.commands.primitive.gripper.setGripperState;
import frc.robot.commands.primitive.orientation.intakeCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PutConeOnSecondGrid extends SequentialCommandGroup {
  private static Arm arm;
  /** Creates a new PutConeOnSecondGrid. */
  public PutConeOnSecondGrid() {
    arm = Arm.getInstance();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(arm);
    addCommands
    (new setGripperState(SequenceType.Cone),
    new Wait(0.25),
    new calibrateArm(),
    new RotateArmToPoint(133),
    new extendArmToLength(51),
    new Wait(1),
    new setGripperState(SequenceType.Off),
    new Wait(0.25),
    new extendArmToLength(0),
    new RotateArmToPoint(0),
    new extendArmToLength(25)
    );
    
  }
}
