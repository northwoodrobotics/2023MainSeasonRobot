package frc.robot.commands.ActionCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class DriveToTag extends CommandBase{
    private final SwerveSubsystem m_Swerve;
    private final PhotonCams m_Cameras;
    private PathPlannerTrajectory Route2Tag;
    private Pose2d TagPose;
    private Transform2d robotToTag;
    private Timer posetimer;
    
    

    public DriveToTag(SwerveSubsystem m_SwerveSubsystem, PhotonCams camera){
        this.m_Swerve = m_SwerveSubsystem;
        this.m_Cameras = camera;
        posetimer = new Timer();
    }
    @Override
    public void initialize(){
        posetimer.reset();
        posetimer.start();
        // gets location of the target relative to the robot's internal position
        TagPose = m_Cameras.getTagLocation(m_Swerve.dt.getPose());

        // calculates a 2d vector in between the two tags
        robotToTag = new Transform2d(m_Swerve.dt.getPose(), TagPose);

        // feeds all data into path generation software
        Route2Tag = PathPlanner.generatePath(
            // these are acceleration and velocity constraints, in m/s and m/s squared
            new PathConstraints(6, 4), 
            // PathPoints have 3 values, the cordinates of the intial point, the heading of the desired vector, and the "holonomic rotation" of the robot
            new PathPoint(m_Swerve.dt.getPose().getTranslation(),robotToTag.getRotation(),m_Swerve.dt.getGyroscopeRotation() ), 
            new PathPoint(TagPose.getTranslation(), robotToTag.getRotation(), TagPose.getRotation())
            );
        
    }

    @Override
    public void execute(){
        m_Swerve.dt.goToPose(Route2Tag.getEndState());
      
    }


    @Override
    public void end(boolean interrupted){
        
    }


}
    

