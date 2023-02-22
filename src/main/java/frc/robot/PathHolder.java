package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.util.Units;

public class PathHolder {
    public static PathPlannerTrajectory SquiglySquare = PathPlanner.loadPath("SquiglySquare",new PathConstraints(Units.feetToMeters(5), Units.feetToMeters(5/2)), false);


}
