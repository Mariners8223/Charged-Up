package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.primitive.extendAndRotateArmToMetersDeg;
import frc.robot.commands.primitive.extendArmToLengthMetersCommand;
import frc.robot.commands.primitive.rotateArmToAngleCommand;
import frc.robot.commands.primitive.useGripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripperV2.GripperV2;
import frc.util.SequenceType;

public class pickup extends SequentialCommandGroup {
  GripperV2 gripper;
  Arm arm;
  public pickup() {
    gripper = GripperV2.getInstance();
    arm = Arm.getInstance();
    addCommands(
      new useGripper(SequenceType.Cone),
      new rotateArmToAngleCommand(-32),
      new useGripper(SequenceType.Cube),
      new extendArmToLengthMetersCommand(17),
      new useGripper(SequenceType.Cone),
      new extendArmToLengthMetersCommand(0),
      new rotateArmToAngleCommand(90),
      new extendArmToLengthMetersCommand(40)
    );
  }
}
