package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivebase;

public class FollowTrajectory extends SequentialCommandGroup {

  public FollowTrajectory(Drivebase drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
    addRequirements(drivebase);
    if (resetOdometry)
      drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
    addCommands(
        new PPSwerveControllerCommand(
            trajectory,
            drivebase::getPose,
            Drivetrain.xAutoPID.createPIDController(),
            Drivetrain.yAutoPID.createPIDController(),
            Drivetrain.angleAutoPID.createPIDController(),
            drivebase::setChassisSpeeds,
            drivebase));
  }
}
