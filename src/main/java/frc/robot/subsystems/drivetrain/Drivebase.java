package frc.robot.subsystems.drivetrain;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;

public class Drivebase extends SubsystemBase {
  private static Drivebase instance;

  public enum controlMode {
    robotOriented, fieldOriented
  };

  /**
   * Enumeration of the wheels on the vehicle.
   */
  public enum wheels {
    leftFront, rightFront,
    leftBack, rightBack
  };

  private SwerveModule[] swerveModules;
  private SwerveModuleState[] moduleStates;

  private AHRS NavX;

  private controlMode driveMode;

  private PIDController thetaPIDController;

  private SwerveDriveKinematics swerveKinematics;

  private ChassisSpeeds targetSpeeds;
  private SwerveDriveOdometry odometry;

  private Field2d the_Field2d;

  private Rotation2d angle;
  private Rotation2d m_desiredAngle;

  public boolean boost = false;

  private Drivebase() {
    swerveModules = new SwerveModule[4];
    swerveModules[wheels.leftFront.ordinal()] = new SwerveModule(Drivetrain.FLModule);
    swerveModules[wheels.rightFront.ordinal()] = new SwerveModule(Drivetrain.FRModule);
    swerveModules[wheels.leftBack.ordinal()] = new SwerveModule(Drivetrain.BLModule);
    swerveModules[wheels.rightBack.ordinal()] = new SwerveModule(Drivetrain.BRModule);

    the_Field2d = new Field2d();

    moduleStates = new SwerveModuleState[4];


    NavX = new AHRS();

    swerveKinematics = Drivetrain.swerveKinematics;
    targetSpeeds = new ChassisSpeeds(0, 0, 0);
    driveMode = controlMode.fieldOriented;

    calibrate();


    odometry = new SwerveDriveOdometry(swerveKinematics, Rotation2d.fromDegrees(0), getModulePositions());

    new Trigger(RobotState::isEnabled).onTrue(new StartEndCommand(() -> {
      for (SwerveModule swerveModule : swerveModules) { 
        swerveModule.setNeutralMode(true);
      }
    }, () -> {
      for (SwerveModule swerveModule : swerveModules) {
        swerveModule.setNeutralMode(false);
      }
    }));

    thetaPIDController = Drivetrain.thetaPIDController.createPIDController();
    thetaPIDController.setTolerance(Drivetrain.thetaPIDController.getTolerance());
    thetaPIDController.enableContinuousInput(-360, 360);

    m_desiredAngle = Rotation2d.fromDegrees(NavX.getAngle());

    // CommandScheduler.getInstance().registerSubsystem(this);
    SmartDashboard.putBoolean("boost", boost);
  }

  /**
   * Returns the current pose of the robot in meters.
   * @return The current pose of the robot in meters.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
    // return new Pose2d();
  }

  /**
   * Resets the odometry to the given initial pose.
   * @param angle The angle of the robot in radians.
   * @param modulePositions The positions of the modules in the robot.
   * @param initalPose2d The initial pose of the robot.
   */
  public void resetOdometry(Pose2d initalPose2d) {
    resetOdometry(angle,
        getModulePositions(),
        initalPose2d);
  }

  /**
   * Returns the current module positions for the swerve drivetrain.
   * @return The current module positions for the swerve drivetrain.
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
        swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition() };
  }

  /**
   * Resets the odometry to the given pose and gyro angle.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current module positions.
   * @param pose The current pose.
   */
  public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    odometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  /**
   * Calibrates the gyroscope.
   */
  public void calibrate() {
    NavX.calibrate();
  }

  /**
  * Resets the state of the robot to the given position.
  * @param position the position to reset the robot to
   */
  public void reset(Pose2d position) {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].reset();
      moduleStates[wheel.ordinal()] = new SwerveModuleState();
    }

    // prototype.reset();

    NavX.reset();
    NavX.setAngleAdjustment(position.getRotation().getDegrees());
    odometry.resetPosition(Rotation2d.fromDegrees(NavX.getAngle()), getModulePositions(), position);

    targetSpeeds = new ChassisSpeeds();

    angle = new Rotation2d();

  }

  /**
   * Returns the singleton instance of the Geyser class.
   * @return The singleton instance of the Geyser class.
   */
  public static Drivebase getInstance() {
    if (instance == null)
      instance = new Drivebase();
    return instance;
  }

  public Rotation2d getDesiredAngle() {return m_desiredAngle;}
  public double calculateReference(Rotation2d currentAngle) {return thetaPIDController.calculate(currentAngle.getDegrees(), m_desiredAngle.getDegrees());}
  public double calculateError() {return thetaPIDController.getPositionError(); }

  /**
   * Updates the state of the swerve modules and the odometry.
   */
  public void update() {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].update();
      moduleStates[wheel.ordinal()] = swerveModules[wheel.ordinal()].getCurrentState();
    }

    angle = Rotation2d.fromDegrees(NavX.getAngle());
    odometry.update(angle, getModulePositions());
    SmartDashboard.putNumber("chassis angle", angle.getDegrees());
  }

  /**
   * Sets the control mode of the player.
   * @param driveMode The control mode to set.
   */
  public void setControlMode(controlMode driveMode) {
    this.driveMode = driveMode;
  }

  public float[] getVelocityReal() {
    float speedX = NavX.getVelocityX();
    float SpeedY = NavX.getVelocityY();
    float speedZ = NavX.getVelocityZ();
    float[] speeds = {speedX, SpeedY, speedZ};
    return speeds;
  }

  /**
  * Sets the target speeds for the chassis.
  * @param target_speeds the target speeds for the chassis.
   */
  public void setChassisSpeeds(ChassisSpeeds target_speeds) {
    this.targetSpeeds = target_speeds;

    SwerveModuleState[] desiredStates = swerveKinematics.toSwerveModuleStates(target_speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
  }

  public void SetChassisAngle(double angle){
    
  }

  /**
   * Drives the robot using the given speed and rotation.
   *
   * @param xSpeed The speed in the x direction.
   * @param ySpeed The speed in the y direction.
   * @param rot The rotation.
   */
  public void drive(double ySpeed, double xSpeed, double rot, Rotation2d angle) {
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    m_desiredAngle = angle;
    SwerveModuleState[] desiredStates;
    switch (driveMode) {
      case fieldOriented:
        this.targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(getAngleDegrees()));
        desiredStates = swerveKinematics
            .toSwerveModuleStates(this.targetSpeeds);
        break;
      default:
        this.targetSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        desiredStates = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        break;
    }
    // System.out.println(this.targetSpeeds);
    // System.out.println(desiredStates);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.freeSpeedMetersPerSecond);
    // System.out.println(desiredStates[0]);
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].set(desiredStates[wheel.ordinal()]);
    }
    // SwerveModuleState[] desiredStates =
    // swerve_kinematics.toSwerveModuleStates(field_oriented_target_speeds)
  }

  /**
   * Stops all the swerve modules.
   */
  public void stop() {
    for (wheels wheel : wheels.values()) {
      swerveModules[wheel.ordinal()].stop();
    }

    // prototype.stop();
  }

  /**
   * Returns the angle of the player's head.
   * @return the angle of the player's head, in degrees
   */
  public double getAngle() {
    return angle.getDegrees();
  }

  /**
   * Resets the gyroscope's heading to zero.
   */
  public void resetGyro() {
    NavX.reset();
    thetaPIDController.setSetpoint(0);
    thetaPIDController.reset();
  }

  public double getRotationPID(double target) {
    return thetaPIDController.calculate(getAngle(), target);
  }

  public boolean isAtRotationSetpoint() {
    return thetaPIDController.atSetpoint();
  }

  public Rotation2d getAngleRotation() {
    return angle;
  }

  public double getRoll(){
    return NavX.getRoll();
  }

  public double getAngleDegrees() { return NavX.getAngle(); }

  public double getPitch() { return NavX.getPitch(); }

  public Command followTrajectory(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          if (isFirstPath) {
            this.resetOdometry(trajectory.getInitialHolonomicPose());
            NavX.setAngleAdjustment(trajectory.getInitialHolonomicPose().getRotation().getDegrees());
          }
        }),
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            Drivetrain.xAutoPID.createPIDController(),
            Drivetrain.yAutoPID.createPIDController(),
            Drivetrain.angleAutoPID.createPIDController(),
            this::setChassisSpeeds,
            this),
        new InstantCommand(() -> {
          this.stop();
        }));
  }

  public void toogleboost(){
    boost = !boost;
  }

  @Override
  public void periodic() {
    update();
    SmartDashboard.putData(NavX);
    SmartDashboard.putNumber("pitch", getRoll());
    SmartDashboard.putNumber("RealSpeedX", getVelocityReal()[0]);
    SmartDashboard.putNumber("RealSpeedY", getVelocityReal()[1]);
    SmartDashboard.putNumber("RealSpeedZ", getVelocityReal()[2]);
    SmartDashboard.getBoolean("boost", boost);
  }
}