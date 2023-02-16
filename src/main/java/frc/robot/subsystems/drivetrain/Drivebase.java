package frc.robot.subsystems.drivetrain.module;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drivetrain.gyro.GyroIONavX2;

public class Drivebase extends SubsystemBase {

  private static Drivebase instance;

  private GyroIONavX2 gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private ModuleIOFXAndSMax[] swerveModules = new ModuleIOFXAndSMax[4];
  private ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] { new ModuleIOInputsAutoLogged(),
      new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(),
      new ModuleIOInputsAutoLogged() };

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private boolean brakeMode = false;

  private Drivebase(GyroIONavX2 gyroIO, ModuleIOFXAndSMax[] Modules) {
    swerveModules[0] = Modules[0];
    swerveModules[1] = Modules[1];
    swerveModules[2] = Modules[2];
    swerveModules[3] = Modules[3];
    this.gyroIO = gyroIO;

    kinematics = new SwerveDriveKinematics(getModuleTranslations());
    odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroIO.getAngleDeg()), new SwerveModulePosition[] {
      swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
      swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition()
    });
  }

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase(GyroIONavX2.getInstance(),
          new ModuleIOFXAndSMax[] { new ModuleIOFXAndSMax(Drivetrain.TLModule),
              new ModuleIOFXAndSMax(Drivetrain.TRModule),
              new ModuleIOFXAndSMax(Drivetrain.BLModule), new ModuleIOFXAndSMax(Drivetrain.BRModule) });
    }
    ;
    return instance;
  }

  public void Drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
        fieldRelative && gyroIO.isConnected()
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(gyroIO.getAngleDeg()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
        Drivetrain.SwerveModuleConstants.freeSpeedMetersPerSecond);
    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].setDesiredState(moduleStates[i]);
    }
  }

  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
        Constants.Drivetrain.TRModule.position,
        Constants.Drivetrain.TLModule.position,
        Constants.Drivetrain.BRModule.position,
        Constants.Drivetrain.BLModule.position
    };
  }

  @Override
  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].updateInputs(moduleInputs[i]);
      swerveModules[i].setDriveBrakeMode(brakeMode);
      Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
          moduleInputs[i]);
    }

    odometry.update(getRotation2d(), new SwerveModulePosition[] {
        swerveModules[0].getModulePosition(), swerveModules[1].getModulePosition(),
        swerveModules[2].getModulePosition(), swerveModules[3].getModulePosition()
    });
    Logger.getInstance().recordOutput("Odometry", getPose());

  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyroIO.getAngleDeg());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  
  public boolean getBrake() {
    return this.brakeMode;
  }

  public void setBrkae(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }
}
