// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.basic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.extendArmToLength;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToGroundGrid extends SequentialCommandGroup {
  /** Creates a new MoveArmToGroundGrid. */
  public MoveArmToGroundGrid() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateArmToPoint(40),
      new extendArmToLength(17)
    );
  }
}
