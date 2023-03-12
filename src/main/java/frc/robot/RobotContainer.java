// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import javax.swing.text.Position;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.commands.Autonomous.Autos;
import frc.robot.commands.Autonomous.BalanceOnRamp;
import frc.robot.commands.Autonomous.EnterRamp;
import frc.robot.commands.Autonomous.PutConeOnSecondGrid;
import frc.robot.commands.primitive.Wait;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.SetArmPostion;
import frc.robot.commands.primitive.arm.calibrateArm;
import frc.robot.commands.primitive.arm.extendArmToLength;
import frc.robot.commands.primitive.arm.testArm;
import frc.robot.commands.primitive.arm.testArmHigh;
import frc.robot.commands.primitive.gripper.setGripperState;
import frc.robot.commands.primitive.orientation.intakeCommand;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.OldLimeLight;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivebase;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.orientation.Orientation;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;
import frc.util.humanIO.CommandPS5Controller;
import frc.util.humanIO.JoystickAxis;
import frc.robot.commands.primitive.arm.MoveArmToSetPoint;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  // Triggers



  // Controller
  private static final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private static final CommandPS5Controller subController = new CommandPS5Controller(1);
  private static final JoystickAxis driveR2Trigger = new JoystickAxis(driveController, 4);
  private static final JoystickAxis driveL2Trigger = new JoystickAxis(driveController, 3);
  private static int position;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Pneumatics.getInstance();
    Gripper.getInstance();
    Pneumatics.getInstance().enableCompressor();
    LimeLight.getInstance();
    Orientation.getInstance();
    Gripper.getInstance().solenoidOff();


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

    Drivebase.getInstance().setDefaultCommand(new RunCommand(() -> {
      Drivebase.getInstance().drive(RobotContainer.calculateDeadband(-getDriveControllerRawAxis(0)) * SwerveModuleConstants.freeSpeedMetersPerSecond,
          RobotContainer.calculateDeadband(getDriveControllerRawAxis(1)) * SwerveModuleConstants.freeSpeedMetersPerSecond, RobotContainer.calculateDeadband(-getDriveControllerRawAxis(2)) * 10);
    }, Drivebase.getInstance()));



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
  //   // controller.cross().onTrue(new toggleOrienationMotors(0.6));
  //   // controller.square().onTrue(new toggleRampSolenoid());
  //   // controller.circle().onTrue(new setGripperState(SequenceType.Cone));
  //   // controller.triangle().onTrue(new setGripperState(SequenceType.Cube));
  //   // controller.options().onTrue(new setGripperState(SequenceType.Off));

  //   // R2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));
  //   // L2Trigger.onTrue(new InstantCommand(() -> arm.setFalconPO(-0.5))).onFalse(new InstantCommand(() -> arm.setFalconPO(0)));

  //   controller.cross().onTrue(new InstantCommand(() -> Drivebase.getInstance().resetGyro()));
  //   controller.triangle().onTrue(new InstantCommand(() -> Orientation.getInstance().lowerRamp()));
  //   controller.square().onTrue(new SequentialCommandGroup(new EnterRamp(), new BalanceOnRamp()));
  //   // controller.L1().onTrue(new StartEndCommand(() -> Gripper.getInstance().solenoidBack(), () -> Gripper.getInstance().solenoidForward()));
  //   // controller.R1().onTrue(new StartEndCommand(() -> Gripper.getInstance().solenoidBack(), () -> Gripper.getInstance().solenoidOff()));
  //   //controller.L1().onTrue(new InstantCommand(() -> Gripper.getInstance().solenoidForward()))
  //       // .onFalse(new InstantCommand(() -> Gripper.getInstance().solenoidBack()));
  //   //controller.R1().onTrue(new InstantCommand(() -> Gripper.getInstance().solenoidForward()))
  //      // .onFalse(new InstantCommand(() -> Gripper.getInstance().solenoidOff()));

  //   // L2Trigger.onTrue(new intakeCommand(0.6)).onFalse(new InstantCommand(() -> { Orientation.getInstance().raiseOrientation(); Orientation.getInstance().stop();}));
  //   // R2Trigger.onTrue(new intakeCommand(-0.6)).onFalse(new InstantCommand(() -> { Orientation.getInstance().raiseOrientation(); Orientation.getInstance().stop();}));


  //   //controller.L2().whileTrue(new intakeCommand(0.6));
  //  // controller.R2().whileTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(-0.6)))
  //      // .onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
      
  //   //controller.triangle().onTrue(new InstantCommand(() -> {
  //   // Orientation.getInstance().setSpeed(0.6);
  //  // })).onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));

  //   //controller.square().onTrue(new testArm());
  //  //controller.touchpad().onTrue(new testArmHigh());
  //   controller.options().onTrue(new SequentialCommandGroup(
  //     new calibrateArm(),
  //     new Wait(0.25),
  //     new RotateArmToPoint(0),
  //     new setGripperState(SequenceType.Off),
  //     new extendArmToLength(27)
  //   ));

  //   // R2Trigger.onTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(0.6)))
  //   //     .onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
  //   // L2Trigger.onTrue(new InstantCommand(() ->  Orientation.getInstance().lowerRamp()))

  //   // controller.circle().whileTrue(new InstantCommand(() -> Arm.getInstance().set775PO(0.5))).onFalse(new InstantCommand(() -> Arm.getInstance().stopExtensionMotor()));
  //   // controller.square().whileTrue(new InstantCommand(() -> Arm.getInstance().set775PO(-0.5))).onFalse(new InstantCommand(() -> Arm.getInstance().stopExtensionMotor()));
  //   // controller.povUp().whileTrue(new InstantCommand(() -> Arm.getInstance().setFalconPO(0.5))).onFalse(new InstantCommand(() -> Arm.getInstance().stopRotationMotor()));
  //   // controller.povDown().whileTrue(new InstantCommand(() -> Arm.getInstance().setFalconPO(-0.5))).onFalse(new InstantCommand(() -> Arm.getInstance().stopRotationMotor()));
  //   // controller.povDown().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
  //   // controller.L1().onTrue(new toggleOrienationSoleniod());
  //   // controller.R1().onTrue(new toggleOrienationSoleniod());
  //   // controller.povRight().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 0));
  //   // controller.povUp().onTrue(new ExtendOrRotateArm(SequenceType.Arm, 91));
  //   // controller.povLeft().onTrue(new ExtendOrRotateArm(SequenceType.Arm, -17));
  //   // controller.povUp().onTrue(new RotateArmToPoint(100));
  //   // controller.povUpLeft().onTrue(new RotateArmToPoint(80));
  //   // controller.povLeft().onTrue(new RotateArmToPoint(45));
  //   // controller.povDown().onTrue(new RotateArmToPoint(0));
  //   // controller.povDownLeft().onTrue(new RotateArmToPoint(-15));
  //   // controller.circle().onTrue(new extendArmToLength(30));
  //   // controller.square().onTrue(new extendArmToLength(0));
    
  //   controller.povDown().onTrue(new extendArmToLength(10));
  //   controller.povRight().onTrue(new extendArmToLength(0));
  //   controller.povLeft().onTrue(new extendArmToLength(20));
  //   controller.povUp().onTrue(new calibrateArm());
  //   controller.share().onTrue(new PutConeOnSecondGrid());
  //       // .onFalse(new InstantCommand(() -> Arm.getInstance().extendToLengthMeters(2)));




    driveController.cross().onTrue(new InstantCommand(() -> Drivebase.getInstance().resetGyro()));
    driveController.triangle().whileTrue(new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0.5))).onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0)));
    driveController.L1().whileTrue(new intakeCommand(0.6));
    driveController.L2().onTrue(new InstantCommand(() -> Orientation.getInstance().lowerOrientation())); driveController.L2().onFalse(new InstantCommand(() -> Orientation.getInstance().raiseOrientation()));
    driveController.L2().onTrue(new InstantCommand(() -> Orientation.getInstance().lowerRamp())); driveController.L2().onFalse(new InstantCommand(() -> Orientation.getInstance().raiseRamp()));
    driveController.R1().onTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(0.6))); driveController.R1().onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
    driveController.R2().onTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(-0.6))); driveController.R2().onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
    



    subController.povUp().onTrue(new SetArmPostion(true));
    subController.povDown().onTrue(new SetArmPostion(false));
    subController.povRight().onTrue(new MoveArmToSetPoint());
    subController.povLeft().onTrue(new calibrateArm());
    subController.L1().onTrue(new setGripperState(SequenceType.Cone)); subController.L1().onFalse(new setGripperState(SequenceType.Off));
    subController.R1().onTrue(new setGripperState(SequenceType.Cube)); subController.R1().onFalse(new setGripperState(SequenceType.Off));
    subController.cross().onTrue(new SequentialCommandGroup(new extendArmToLength(0), new RotateArmToPoint(0), new extendArmToLength(23)));

    //TODO- add manual adjustment



  }


  public static double getDriveControllerRawAxis(int axis){
    return driveController.getRawAxis(axis);
  }

  public static double getSubControllerRawAxis(int axis){
    return subController.getRawAxis(axis);
  }


  public static CommandPS5Controller getDriveController(){
    return driveController;
  }

  public static CommandPS5Controller getSubController(){
    return subController;
  }

  public static  int getArmPosition(){
    return position;
  }

  public static void setArmPostion(int Position){
    position = Position;
  }

  public static JoystickAxis getDriveR2JoystickAxis() { return driveR2Trigger; }
  public static JoystickAxis getDriveL2JoystickAxis() { return driveL2Trigger; }

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
