// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.PickupToUpperCone;
import frc.robot.commands.primitive.extendAndRotateArmToMetersDeg;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.primitive.extendArmToLengthMetersCommand;
import frc.robot.commands.primitive.minorAdjustExtend;
import frc.robot.commands.primitive.minorAdjustRotaion;
import frc.robot.commands.primitive.toggleOrientaionSolenoid;
import frc.robot.commands.primitive.toggleRampSolenoid;
import frc.robot.commands.primitive.rotateArmToAngleCommand;
import frc.robot.commands.primitive.setOrienationSpeed;
import frc.robot.commands.primitive.useGripper;
import frc.robot.subsystems.Tank;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.gripperV2.GripperV2;
import frc.robot.subsystems.orientation.Orientation;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.humanIO.CommandPS5Controller;

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
  GripperV2 gripperV2 = GripperV2.getInstance();
  Tank tank = Tank.getinstance();



  // Controller
  private static final CommandPS5Controller controller = new CommandPS5Controller(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");
  private static boolean inverted = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Pneumatics.getInstance();
    GripperV2.getInstance();
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
    controller.cross().whileTrue(new setOrienationSpeed(0.6));
    controller.share().whileTrue(new setOrienationSpeed(0.3));
    controller.triangle().onTrue(new PickupToUpperCone());
    controller.square().onTrue(new toggleRampSolenoid());
    controller.circle().onTrue(new useGripper(SequenceType.Cone));
    controller.options().onTrue(new useGripper(SequenceType.off));
    controller.L1().onTrue(new extendArmToLengthMetersCommand(40));
    controller.R1().onTrue(new extendArmToLengthMetersCommand(0));
    controller.povDown().onTrue(new rotateArmToAngleCommand(0));
    controller.povUp().onTrue(new rotateArmToAngleCommand(90));
    controller.povRight().onTrue(new rotateArmToAngleCommand(70));
    controller.L2().onTrue(new minorAdjustRotaion());
    controller.R2().whileTrue(new minorAdjustExtend());

  }

  public static double getRawAxis(int axis){
    return controller.getRawAxis(axis);
  }

  public static int toggleinvert(){
    if(controller.touchpad().getAsBoolean()){
      if(inverted){
        inverted = false;
        return 1;
      }
      else{
        inverted = true;
        return 2;
      }
    }
    return 3;
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
