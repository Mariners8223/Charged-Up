package frc.robot.commands.Autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Drivetrain.SwerveModuleConstants;
import frc.robot.subsystems.drivetrain.Drivebase;

public final class Autos {

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command onePieceAuto() {
    List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("1PiecePath",
        SwerveModuleConstants.autoSpeedMetersPerSec, SwerveModuleConstants.autoMaxAccelerationPerSecSquared);
    HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("event1", new PrintCommand("passed marker 1"));
      eventMap.put("event2", new PrintCommand("passed marker 2"));
    return new SequentialCommandGroup(
        new FollowPathWithEvents(Drivebase.getInstance().followTrajectory(trajectories.get(0), true),
            trajectories.get(0).getMarkers(), eventMap),
        new PrintCommand("finished path")
    );
  }
}
