// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Autonomous.Autos;
import frc.robot.commands.primitive.orientation.intakeCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivebase;
import frc.robot.subsystems.gripper.Gripper;
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
        break;
    }

    // Drivebase.getInstance().setDefaultCommand(new RunCommand(() -> {
    //   Drivebase.getInstance().drive(calculateDeadband(controller.getRawAxis(0)) * SwerveModuleConstants.freeSpeedMetersPerSecond,
    //       calculateDeadband(-controller.getRawAxis(1)) * SwerveModuleConstants.freeSpeedMetersPerSecond, calculateDeadband(controller.getRawAxis(2)) * 10);
    // }, Drivebase.getInstance()));

    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Drive 2m", Autos.exampleAuto(Drivebase.getInstance()));
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
    // controller.cross().onTrue(new toggleOrienationMotors(0.6));
    // controller.square().onTrue(new toggleRampSolenoid());
    // controller.circle().onTrue(new setGripperState(SequenceType.Cone));
    // controller.triangle().onTrue(new setGripperState(SequenceType.Cube));
    // controller.options().onTrue(new setGripperState(SequenceType.Off));

    // R2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));
    // L2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(-0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));

    controller.cross().onTrue(new InstantCommand(() -> Drivebase.getInstance().resetGyro()));
    // controller.L1().onTrue(new StartEndCommand(() -> Gripper.getInstance().solenoidBack(), () -> Gripper.getInstance().solenoidForward()));
    // controller.R1().onTrue(new StartEndCommand(() -> Gripper.getInstance().solenoidBack(), () -> Gripper.getInstance().solenoidOff()));
    controller.L1().onTrue(new InstantCommand(() -> Gripper.getInstance().solenoidBack()))
        .onFalse(new InstantCommand(() -> Gripper.getInstance().solenoidForward()));
    controller.R1().onTrue(new InstantCommand(() -> Gripper.getInstance().solenoidBack()))
        .onFalse(new InstantCommand(() -> Gripper.getInstance().solenoidOff()));

    L2Trigger.onTrue(new intakeCommand());
    // controller.povLeft().onTrue(new InstantCommand(() -> arm.set775PO(0.5))).onFalse(new InstantCommand(() -> arm.set775PO(0)));
    // controller.povRight().onTrue(new InstantCommand(() -> arm.set775PO(-0.5))).onFalse(new InstantCommand(() -> arm.set775PO(0)));
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

  public static CommandPS5Controller getCotroller() {
    return controller;
  }

  public static JoystickAxis getR2JoystickAxis() { return R2Trigger; }
  public static JoystickAxis getL2JoystickAxis() { return L2Trigger; }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static double calculateDeadband(double value) {
    if (Math.abs(value) < 0.2) return 0;
    return value;
  }
}
