package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveSubsystem;
import frc.ExternalLib.GrassHopperLib.BetterSwerveModuleState;
import frc.ExternalLib.GrassHopperLib.SecondOrderKinematics;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
//import frc.ExternalLib.SpectrumLib.controllers.SpectrumXboxController;

public class TeleopDriveCommand extends CommandBase {
  // transfers X, Y and Rotation commands to drivetrain. Uses the plug and play
  // libary for this.
  private final SwerveSubsystem m_SwerveSubsystem;

  private final DoubleSupplier m_translationXSupplier;
  private final DoubleSupplier m_translationYSupplier;
  private final DoubleSupplier m_rotationSupplier;

  public TeleopDriveCommand(SwerveSubsystem subsystem, DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    this.m_SwerveSubsystem = subsystem;
    this.m_translationXSupplier = translationXSupplier;
    this.m_translationYSupplier = translationYSupplier;
    this.m_rotationSupplier = rotationSupplier;

    // this.drivecontroller = RobotContainer.driveController;

    addRequirements(subsystem);

  }

  @Override
  public void execute() {
    // x and  movement is negated so that direction of travel is the same sign as localizaiton.  
    m_SwerveSubsystem.dt.driveClean(-m_translationXSupplier.getAsDouble()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS, -m_translationYSupplier.getAsDouble()*Constants.DriveConstants.MAX_STRAFE_SPEED_MPS, m_rotationSupplier.getAsDouble());
   /* m_SwerveSubsystem.dt.setModuleStates(
    DriveConstants.KINEMATICS.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      m_translationXSupplier.getAsDouble()*DriveConstants.MAX_FWD_REV_SPEED_MPS,
       m_translationYSupplier.getAsDouble()*DriveConstants.MAX_FWD_REV_SPEED_MPS, 
       m_rotationSupplier.getAsDouble(), 
       m_SwerveSubsystem.dt.getGyroscopeRotation())) 
    );*/
  }

  @Override
  public void end(boolean interrupted) {
  

  }

}
