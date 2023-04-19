// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.commands.Autonomous.AligenToConeGrid;
import frc.robot.commands.Autonomous.Autos;
import frc.robot.commands.Autonomous.BalanceOnRamp;
import frc.robot.commands.Autonomous.EnterRamp;
import frc.robot.commands.Autonomous.PutConeOnSecondGrid;
import frc.robot.commands.Autonomous.StartEnterRamp;
import frc.robot.commands.*;
import frc.robot.commands.primitive.arm.MoveArmToSetPoint;
import frc.robot.commands.primitive.arm.RotateArmToPoint;
import frc.robot.commands.primitive.arm.SetArmPostion;
import frc.robot.commands.primitive.arm.calibrateArmExtension;
import frc.robot.commands.primitive.arm.extendArmToLength;
import frc.robot.commands.primitive.gripper.setGripperPostion;
import frc.robot.commands.primitive.orientation.intakeCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivebase;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.orientation.Orientation;
import frc.robot.subsystems.pneumatics.Pneumatics;
import frc.util.SequenceType;
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

  // Triggers



  // Controller
  private static final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private static final CommandPS5Controller subController = new CommandPS5Controller(1);
  private static final JoystickAxis driveR2Trigger = new JoystickAxis(driveController, 4);
  private static final JoystickAxis driveL2Trigger = new JoystickAxis(driveController, 3);
  private static int position;
  private static SequenceType moveType = SequenceType.Arm;
  private double SpeedX = 0;
  private double SpeedY= 0;
  private double Rotation= 0;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DataLogManager.start();
    Pneumatics.getInstance();
    Gripper.getInstance();
    Pneumatics.getInstance().enableCompressor();
    Arm.getInstance();
    //LimeLight.getInstance();
    Orientation.getInstance();
    Gripper.getInstance().solenoidOff();
    SmartDashboard.putString("Manual Adjust Select", "Rotation");
    SmartDashboard.putString("current Postion", "Home");
    SmartDashboard.putString("Future Postion", "Home");
    Drivebase.getInstance().resetGyro();


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




    // Set up auto routines
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Cone on High Grid and Balance", Autos.oneConeAndBalance());
    autoChooser.addOption("Cone on High", new PutConeOnSecondGrid());
    autoChooser.addOption("Balance", new SequentialCommandGroup(new StartEnterRamp(), new EnterRamp(), new BalanceOnRamp()));
    autoChooser.addOption("aliign", new AligenToConeGrid());
    // Configure the button bindings

    // Drivebase.getInstance().setDefaultCommand(new RunCommand(() -> {
    //   Drivebase.getInstance().drive(-RobotContainer.calculateDeadband(getDriveControllerRawAxis(0)) * SwerveModuleConstants.freeSpeedMetersPerSecond,
    //       RobotContainer.calculateDeadband(getDriveControllerRawAxis(1)) * SwerveModuleConstants.freeSpeedMetersPerSecond, RobotContainer.calculateDeadband(getDriveControllerRawAxis(2)) * -12.5);
    // }, Drivebase.getInstance()));

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
    driveController.R1().onTrue(new InstantCommand(() -> Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond = 2)).onFalse(new InstantCommand(() -> Constants.Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond = 4.5 ));
    driveController.L1().whileTrue(new intakeCommand(0.6));
    driveController.L2().onTrue(new InstantCommand(() -> Orientation.getInstance().lowerOrientation())); driveController.L2().onFalse(new InstantCommand(() -> Orientation.getInstance().raiseOrientation()));
    driveController.L2().onTrue(new InstantCommand(() -> Orientation.getInstance().lowerRamp())); driveController.L2().onFalse(new InstantCommand(() -> Orientation.getInstance().raiseRamp()));
    driveController.povUp().onTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(0.6))); driveController.povUp().onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
    driveController.povDown().onTrue(new InstantCommand(() -> Orientation.getInstance().setSpeed(-0.6))); driveController.povDown().onFalse(new InstantCommand(() -> Orientation.getInstance().stop()));
    

    subController.R1().onTrue(new setGripperPostion(SequenceType.Cone)).onFalse(new setGripperPostion(SequenceType.Off));
    subController.L1().onTrue(new setGripperPostion(SequenceType.Cube)).onFalse(new setGripperPostion(SequenceType.Off));
    subController.povUp().onTrue(new SetArmPostion(true));
    subController.povDown().onTrue(new SetArmPostion(false));
    subController.povRight().onTrue(new MoveArmToSetPoint());
    subController.options().onTrue(new calibrateArmExtension());
    //subController.share().onTrue(new calibrateArmRotation());
    subController.cross().onTrue(new SequentialCommandGroup(
        new extendArmToLength(0),
        new RotateArmToPoint(5),
        new extendArmToLength(17),
        new InstantCommand(() -> setArmPostion(0)),
        new InstantCommand(() -> SmartDashboard.putString("current Postion", "Home"))));
    subController.R2().onTrue(new InstantCommand(() -> setArmPostionOrExtension(0.15)))
        .onFalse(new InstantCommand(() -> setArmPostionOrExtension(0)));
    subController.L2().onTrue(new InstantCommand(() -> setArmPostionOrExtension(-0.15)))
        .onFalse(new InstantCommand(() -> setArmPostionOrExtension(0)));
    subController.triangle().onTrue(new InstantCommand(() -> setMoveType()));
    subController.button(15).onTrue(new SequentialCommandGroup(
        new extendArmToLength(0),
        new RotateArmToPoint(0),
        new extendArmToLength(25),
        new InstantCommand(() -> setArmPostion(0)),
        new InstantCommand(() -> SmartDashboard.putString("current Postion", "Home"))));
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

  private static void setMoveType(){
    String type = "Rotation";
    switch(moveType){
      case Arm:
      
        moveType = SequenceType.Extenion;
        type = "Extension";
        break;
      
      case Extenion:
        moveType = SequenceType.Arm;
        type = "Rotation";
        break;
      default:
        break;
    }
    SmartDashboard.putString("Manual Adjust Select", type);
  }

  private static void setArmPostionOrExtension(double speed){
    if(getArmPosition() == 0 || getArmPosition() == 1){speed = 0;}
    switch(moveType){
      case Arm:
        Arm.getInstance().setFalconPO(speed);

        break;

      case Extenion:
        Arm.getInstance().set775PO(speed);
        break;
      default:
        break;
    }
  }

  public void SetDriveBaseSpeeds(double speedX, double speedY, double roation){
    Rotation = roation;
    SpeedX = speedX;
    SpeedY = SpeedX;
  }

  public double[] getDriveBaseSpeeds(){
    double[] Speeds = new double[2];
    Speeds[0] = SpeedX;
    Speeds[1] = SpeedY;
    Speeds[2] = Rotation;
    return Speeds;
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
