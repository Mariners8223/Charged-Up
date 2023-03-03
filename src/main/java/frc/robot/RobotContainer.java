// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.primitive.arm.ExtendOrRotateArm;
import frc.robot.commands.primitive.arm.InvertManualDirectaion;
import frc.robot.commands.primitive.arm.manualAdjust;
import frc.robot.commands.primitive.gripper.setSolenoidState;
import frc.robot.commands.primitive.gripper.toggleGripperSolenoid;
import frc.robot.commands.primitive.oriantion.toggleOrienationMotors;
import frc.robot.commands.primitive.oriantion.toggleOrienationSoleniod;
import frc.robot.commands.primitive.oriantion.toggleRampSolenoid;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.orientation.Orientation;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.humanIO.CommandPS5Controller;
import frc.util.humanIO.JoystickAxis;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  Arm arm = Arm.getInstance();
  Gripper gripperV2 = Gripper.getInstance();
  Tank tank = Tank.getinstance();



  // Controller
  private static final CommandPS5Controller controller = new CommandPS5Controller(0);
  private static final JoystickAxis R2Trigger = new JoystickAxis(controller, 4);
  private static final JoystickAxis L2Trigger = new JoystickAxis(controller, 3);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private static boolean inverted = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Pneumatics.getInstance();
    Gripper.getInstance();
    Pneumatics.getInstance().enableCompressor();

    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        // drive = new Drive(new DriveIOFalcon500());
        // flywheel = new Flywheel(new FlywheelIOFalcon500());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        break;

      // Replayed robot, disable IO implementations
      default:
        //drive = new Drive(new DriveIO() {
        //});
        break;
    }
    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // Configure the button bindings
    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controller.touchpad().onTrue(new InvertManualDirectaion());
    controller.cross().onTrue(new toggleOrienationMotors(0.6));
    R2Trigger.whileTrue(new manualAdjust(SequenceType.Arm));
    L2Trigger.whileTrue(new manualAdjust(SequenceType.Extenion));
    controller.square().onTrue(new toggleRampSolenoid());
    controller.circle().onTrue(new setSolenoidState(SequenceType.Cone));
    controller.triangle().onTrue(new setSolenoidState(SequenceType.Cube));
    controller.options().onTrue(new setSolenoidState(SequenceType.Off));
    // controller.povDown().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
    // controller.L1().onTrue(new toggleOrienationSoleniod());
    // controller.R1().onTrue(new toggleOrienationSoleniod());
    // controller.povRight().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
    // controller.povUp().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 91));
    // controller.povLeft().onTrue(new ExtendOrRotateArm(SequenceType.Arm, -17));
  }

  public static double getRawAxis(int axis){
    return controller.getRawAxis(axis);
  }

  public static void SetInverted(boolean state){
    inverted = state;
  }

  public static boolean getInverted(){
    return inverted;
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
