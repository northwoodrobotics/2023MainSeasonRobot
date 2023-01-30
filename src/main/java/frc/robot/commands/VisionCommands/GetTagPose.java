package frc.robot.commands.VisionCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonCams;

public class GetTagPose extends CommandBase{
    private final PhotonCams m_cameras;
    private Pose2d TagPose; 
    private Timer posetimer;
    
    public GetTagPose(PhotonCams cameras){
        this.m_cameras = cameras;
        posetimer = new Timer();
     
    }

    @Override
    public void initialize(){
        posetimer.reset();
        posetimer.start();
       
     
    }
    public void execute(){
       
    }
    public void end(boolean interrupted){
         
       SmartDashboard.putNumber("PostVisionPoseX", TagPose.getX());
       SmartDashboard.putNumber("PostVisonPoseY", TagPose.getY());
       SmartDashboard.putNumber ("PostVisonPoseRot", TagPose.getRotation().getDegrees());
       
    }
       @Override
    public boolean isFinished(){
        return posetimer.hasElapsed(.25);
    }

    

}
