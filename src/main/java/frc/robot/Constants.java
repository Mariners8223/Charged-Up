package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.util.PIDFGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;
  public static final double fullThres = 0.99;
  public static final double heldThres = -0.9;
  public static final boolean IsLimeLightAprilTags = false;
  public static final int FALCON500_COUNTS_PER_REVOLUTION = 2048;
  static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
  }

  public static final int MOTORS_CHECKED_PER_TICK = 1;
  public static final int SRX_MAG_COUNTS_PER_REVOLUTION = 1024;
  public static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
  public static final Transform3d robotToLimeLight = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }



  public static class GripperConstants {
    public static final double GRIPPER_KP = 0.1;
    public static final double GRIPPER_SPEED = 1.0;
    public static final double GRIPPER_GEAR_RATIO = 1.6;
    public static final double GRIPPER_TOLERANCE = Units.degreesToRadians(2);
    public static final int[] GRIPPER_DOUBLE_SOLENOID_PORTS = {0, 1};
  }

  public static class OrientationConstants {
    public static final int ORIENTATION_ELEVATED_MOTOR = 21;
    public static final int ORIENTATION_RAMP_MOTOR = 20;
    public static final int[] ORIENTATION_ELEVATED_SOLENOID_PORTS = {2, 3};
    public static final int[] ORIENTATION_RAMP_SOLENOID_PORTS = {4, 5};
  }


  public static class ArmConstants {
    public static final int ARM_ROTATION = 28;
    public static final int EXTENSION_LIMIT_SWITCH_PORT = 0;
    public static final int ARM_EXTENSION = 24;


    public static final double ARM_ROTATION_KP = 0.05;
    // public static final double ARM_ROTATION_KP = 0.0;
    public static final double ARM_ROTATION_KI = 0.0;
    public static final double ARM_ROTATION_KD = 0.0;
    public static final double ARM_ROTATION_KF = 0.0;
    public static final double ARM_REVOLUTIONS_PER_DEGREE = 4432;
    public static final double ARM_ROTATION_TOLERANCE = 1 * 4432;
    public static final double ARM_EXTENSION_KP = 0.8;
    public static final double ARM_EXTENSION_KI = 0.0;
    public static final double ARM_EXTENSION_KD = 0.0;
    public static final double ARM_FORWARD_SOFT_LIMIT = 110 * ARM_REVOLUTIONS_PER_DEGREE;
    public static final double ARM_REVERSE_SOFT_LIMIT = -100 * ARM_REVOLUTIONS_PER_DEGREE;

    public static final double ARM_ROTATION_GEAR_RATIO = 776.25;
    public static final double ARM_EXTENSION_GEAR_RATIO = 2.3;
    public static final double ARM_EXTENSION_CM_PER_REVOLUTION = (24200) / 38.2;

    public static final double ARM_EXTENSION_TOLERANCE = 1 * ARM_EXTENSION_CM_PER_REVOLUTION;


    
  }

  public static final class Drivetrain {
    /**
     * The constants for the Swerve module.
     * 
     * @param idDrive            The ID of the drive motor.
     * @param driveGains         The drive gains.
     * @param idSteering         The ID of the steering motor.
     * @param steeringGains      The steering gains.
     * @param cancoderZeroAngle  The zero angle of the CANCoder.
     * @param canCoderId         The CANCoder ID.
     * @param isSteeringInverted Whether the steering is inverted.
     * @param isDriveInverted    Whether the drive is inverted.
     */
    public static class SwerveModuleConstants {
      public static final double freeSpeedMetersPerSecond = 2.000;
      public static final double driveRatio = 6.75;
      public static final double steeringRatio = 12.5;
      public static final double wheelRadiusMeters = 0.0508; // 2 inches (in meters)
      public static final double wheelCircumferenceMeters = wheelRadiusMeters * 2 * Math.PI;
      public static final double driveDPRMeters = wheelCircumferenceMeters * driveRatio;
      public static final double steeringPositionConversionFactor = 1 / steeringRatio * 360; // degrees / rotation
      public static final double steeringVelocityConversionFactor = steeringPositionConversionFactor / 60; // degrees /
                                                                                                           // (rotations
                                                                                                           // *
                                                                                                           // seconds/minute)
      // public final static double cancoderTLOffset = 0;
      // public final static double cancoderTROffset = 0;
      // public final static double cancoderBLOffset = 0;
      // public final static double cancoderBROffset = 0;

      public final static double cancoderTLOffset = 139.218;
      public final static double cancoderTROffset = 130.253;
      public final static double cancoderBLOffset = 234.404;
      public final static double cancoderBROffset = 179.12109375;

      public final int idDrive;
      public final PIDFGains driveGains;
      public final int idSteering;
      public final PIDFGains steeringGains;
      public final double cancoderZeroAngle;
      public final int canCoderId;
      public final boolean isSteeringInverted;
      public final boolean isDriveInverted;

      public SwerveModuleConstants(int idDrive, int idSteering, double cancoderZeroAngle,
          int canCoderId, boolean isSteeringInverted, boolean isDriveInverted) {
        this(idDrive, idSteering, new PIDFGains(0.05, 0, 0, 0, 1, 0),
            new PIDFGains(0.1, 0, 0.1, 0, 1, 0), cancoderZeroAngle, canCoderId, isSteeringInverted, isDriveInverted);
      }

      public SwerveModuleConstants(int idDrive, int idSteering, PIDFGains driveGains,
          PIDFGains steeringGains, double cancoderZeroAngle, int canCoderId, boolean isSteeringInverted, boolean isDriveInverted) {
        this.idDrive = idDrive;
        this.driveGains = driveGains;
        this.idSteering = idSteering;
        this.steeringGains = steeringGains;
        this.cancoderZeroAngle = cancoderZeroAngle;
        this.canCoderId = canCoderId;
        this.isSteeringInverted = isSteeringInverted;
        this.isDriveInverted = isDriveInverted;
      }
    }

    public static final SwerveModuleConstants FLModule = new SwerveModuleConstants(2, 3,
        SwerveModuleConstants.cancoderTLOffset, 10, false, false);
    public static final SwerveModuleConstants FRModule = new SwerveModuleConstants(4, 5,
        SwerveModuleConstants.cancoderTROffset, 11, false, true);
    public static final SwerveModuleConstants BLModule = new SwerveModuleConstants(6, 7,
        SwerveModuleConstants.cancoderBLOffset, 12, false, false);
    public static final SwerveModuleConstants BRModule = new SwerveModuleConstants(8, 9,
        SwerveModuleConstants.cancoderBROffset, 13, false, false);
  
    public static final PIDFGains xAutoPID = new PIDFGains(0.1, 0.0, 0.0);
    public static final PIDFGains yAutoPID = new PIDFGains(0.1, 0.0, 0.0);
    public static final PIDFGains angleAutoPID = new PIDFGains(0.1, 0.0, 0.0);
    public static final double kTrackWidth = 0.55; // Distance between right and left wheels
    public static final double kWheelBase = 0.55; // Distance between front and back wheels
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

      
  }


}