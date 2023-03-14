package frc.robot.commands.Autonomous;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.drivetrain.Drivebase;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static CommandBase exampleAuto(Drivebase swerve) {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("ExamplePath", new PathConstraints(2.0, 0.5));
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("event 1", new PrintCommand("Passed event 1"));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      swerve::getPose,
      swerve::resetOdometry,
      new PIDConstants(Drivetrain.xAutoPID.getP(), Drivetrain.xAutoPID.getI(), Drivetrain.xAutoPID.getD()),
      new PIDConstants(Drivetrain.angleAutoPID.getP(), Drivetrain.angleAutoPID.getI(), Drivetrain.angleAutoPID.getD()),
      swerve::setChassisSpeeds,
      eventMap,
      swerve
    );

    // return Commands.sequence(autoBuilder.fullAuto(trajectory));
    return autoBuilder.fullAuto(trajectory);
  }

  public static CommandBase  oneConeAndBalance() {
    return new SequentialCommandGroup(
      new PutConeOnSecondGrid(),
      new StartEnterRamp(),
      new EnterRamp(),
      new BalanceOnRamp()
    );
  }
}
