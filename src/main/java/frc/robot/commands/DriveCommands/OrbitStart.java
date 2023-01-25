package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.swervelib.SwerveSubsystem;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class OrbitStart extends CommandBase{
    // does what it says, robot orbits where it orginally started. 
    // will be optomized to do other things, for now this is just a test. 

    private final SwerveSubsystem m_SwerveSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final Pose2d CurrentPose;
    private Transform2d OrbitRadius; 
    private Pose2d Origin = Constants.DriveConstants.DFLT_START_POSE;
  
    public OrbitStart(SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
        DoubleSupplier translationYSupplier,
        DoubleSupplier rotationSupplier) {
      this.m_SwerveSubsystem = subsystem;
      this.m_translationXSupplier = translationXSupplier;
      this.m_translationYSupplier = translationYSupplier;
      this.m_rotationSupplier = rotationSupplier;
      this.CurrentPose = m_SwerveSubsystem.dt.getPose();
      
  
      
  
      addRequirements(subsystem);
  
    }
  
    @Override
    public void execute() {
        OrbitRadius = CurrentPose.minus(Origin);
        
        


  
      m_SwerveSubsystem.dt.setModuleStates(Constants.DriveConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(
              0* Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS,
              OrbitRadius.getTranslation().getDistance(Origin.getTranslation())*m_rotationSupplier.getAsDouble() * Constants.DriveConstants.MAX_STRAFE_SPEED_MPS,
              0* Constants.DriveConstants.MAX_STRAFE_SPEED_MPS
              )));
    }
  
    @Override
    public void end(boolean interrupted) {
     
  
    }
}
