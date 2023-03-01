package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.primitive.extendAndRotateArmToMetersDeg;
import frc.robot.commands.primitive.extendArmToLengthMetersCommand;
import frc.robot.commands.primitive.rotateArmToAngleCommand;
import frc.robot.commands.primitive.toggleOrientaionSolenoid;
import frc.robot.commands.primitive.useGripper;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripperV2.GripperV2;
import frc.robot.subsystems.orientation.Orientation;
import frc.util.SequenceType;

public class PickupToUpperCone extends SequentialCommandGroup {
  GripperV2 gripper;
  Arm arm;
  Orientation orientation;
  public PickupToUpperCone() {
    gripper = GripperV2.getInstance();
    arm = Arm.getInstance();
    orientation = Orientation.getInstance();
    addCommands(
      new toggleOrientaionSolenoid(true, true),
      new useGripper(SequenceType.Cone),
      new rotateArmToAngleCommand(-32),
      new extendArmToLengthMetersCommand(12),
      new useGripper(SequenceType.off),
      new extendArmToLengthMetersCommand(20),
      new useGripper(SequenceType.Cone),
      new extendArmToLengthMetersCommand(0),
      new rotateArmToAngleCommand(95),
      new extendArmToLengthMetersCommand(40)
    );
  }
}
