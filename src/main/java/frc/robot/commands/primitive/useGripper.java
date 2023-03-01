package frc.robot.commands.primitive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.gripperV2.GripperV2;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;

public class useGripper extends CommandBase {
  GripperV2 gripper;
  Pneumatics ph;
  SequenceType type;
  public useGripper(SequenceType type) {
    gripper = GripperV2.getInstance();
    ph = Pneumatics.getInstance();
    this.type = type;
    addRequirements(gripper, ph);
  }

  @Override
  public void initialize() {
    switch (type) {
      case Cone:
        gripper.solenoidForward();
        break;
      
      case Cube:
        gripper.solenoidOff();
        break;
      
      case off:
        gripper.solenoidBack();
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
