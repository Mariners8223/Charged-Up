package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;

public class testArmHigh extends SequentialCommandGroup {
  Arm arm;
  public testArmHigh() {
    arm = Arm.getInstance();
    addRequirements(arm);
    addCommands(
      new extendArmToLength(0),
      new RotateArmToPoint(131),
      new extendArmToLength(45)
    );
  }
}
