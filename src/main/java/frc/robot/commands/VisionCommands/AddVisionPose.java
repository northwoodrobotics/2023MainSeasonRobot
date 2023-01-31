package frc.robot.commands.VisionCommands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveSubsystem;

public class AddVisionPose extends CommandBase{
    private final PhotonCams m_cameras;
    private final SwerveSubsystem m_drivetrain;

    
    public AddVisionPose(PhotonCams cameras, SwerveSubsystem drive){
        this.m_drivetrain = drive;
        this.m_cameras = cameras;
        
     
    }

    @Override
    public void initialize(){

   
       
     
    }
    public void execute(){
        Optional<EstimatedRobotPose> result =
        m_cameras.getEstimatedGlobalPose(m_drivetrain.dt.getPose());

        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            m_drivetrain.dt.VisionPose(
                    camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
          
        }
       
    }
    public void end(boolean interrupted){
         
     
       
    }


    

}
