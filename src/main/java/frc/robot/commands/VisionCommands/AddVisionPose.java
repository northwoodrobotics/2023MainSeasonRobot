package frc.robot.commands.VisionCommands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;

public class AddVisionPose extends CommandBase{
    private final PhotonCams m_cameras;
    // private final SwerveSubsystem m_drivetrain;
 
     
     public AddVisionPose(PhotonCams cameras){
    
         this.m_cameras = cameras;
         addRequirements(cameras);
      
     }
 
     @Override
     public void initialize(){
 
    
        
      
     }
     public void execute(){
         Optional<EstimatedRobotPose> result =
         m_cameras.getEstimatedGlobalPose(RobotContainer.m_SwerveSubsystem.dt.getPose());
 
         if (result.isPresent()) {
             EstimatedRobotPose camPose = result.get();
             RobotContainer.m_SwerveSubsystem.dt.VisionPose(
                     camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
             Logger.getInstance().recordOutput("VisionPose", camPose.estimatedPose.toPose2d());

         }
       
        
     }
     public void end(boolean interrupted){
          
      
        
     }
    

}
