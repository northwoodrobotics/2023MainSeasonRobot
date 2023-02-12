package frc.robot.commands.ActionCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructure.endEffectorState;
import frc.swervelib.SwerveSubsystem;

public class DriveToRamp extends CommandBase{
    private final SwerveSubsystem m_Swerve;
    private final SuperStructure m_SuperStructure;
    private PathPlannerTrajectory Route2Tag;
    private Pose2d TagPose;
    private Transform2d robotToTag;
    private Command pathCommand;
   
    
    

    public DriveToRamp(SwerveSubsystem m_SwerveSubsystem, SuperStructure superStructure){
        this.m_Swerve = m_SwerveSubsystem;
        this.m_SuperStructure = superStructure;
  
        
    }
    @Override
    public void initialize(){
        m_SuperStructure.setSuperStructureState(SuperStructurePresets.ramp);



        // calculates a 2d vector in between the two tags
        robotToTag = new Transform2d(m_Swerve.dt.getPose(), TagPose);

        // feeds all data into path generation software
        Route2Tag = PathPlanner.generatePath(
            // these are acceleration and velocity constraints, in m/s and m/s squared
            new PathConstraints(1, 2), 
            // PathPoints have 3 values, the cordinates of the intial point, the heading of the desired vector, and the "holonomic rotation" of the robot
            new PathPoint(m_Swerve.dt.getPose().getTranslation(),robotToTag.getRotation(),m_Swerve.dt.getGyroscopeRotation() ), 
            new PathPoint(TagPose.getTranslation(), robotToTag.getRotation(), TagPose.getRotation())
            );

            pathCommand=m_Swerve.dt.createCommandForTrajectory(Route2Tag, m_Swerve);
            pathCommand.schedule();
        
    }

    @Override
    public void execute(){
        
       
    }


    @Override
    public void end(boolean interrupted){
        pathCommand.cancel();
        m_Swerve.dt.driveClean(0, 0, 0);
    }


}
    

