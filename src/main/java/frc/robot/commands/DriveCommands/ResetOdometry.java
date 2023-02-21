package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.swervelib.SwerveSubsystem;

public class ResetOdometry extends CommandBase{
        SwerveSubsystem m_swerveSubsystem;
      
        public ResetOdometry(SwerveSubsystem subsystem) {
          this.m_swerveSubsystem = subsystem;
        }
      
        @Override
        public void execute() {
          m_swerveSubsystem.dt.setKnownPose(new Pose2d(new Translation2d(), m_swerveSubsystem.dt.getGyroscopeRotation()));
        }
        @Override 
        public void end(boolean interrupted){
      
        }
        @Override
        public boolean isFinished(){
          return true;
        }
      
    
}
