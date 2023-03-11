package frc.robot.commands.primitive.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;

public class testArm extends SequentialCommandGroup {
  Arm arm;
  public testArm() {
    arm = Arm.getInstance();
    addRequirements(arm);
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new extendArmToLength(0),
      new RotateArmToPoint(117),
      new extendArmToLength(2)
    );
  }
}
