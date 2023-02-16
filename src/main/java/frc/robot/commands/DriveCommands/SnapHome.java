package frc.robot.commands.DriveCommands;

import java.util.function.DoubleSupplier;

import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveSubsystem;

public class SnapHome extends CommandBase{
    private final SwerveSubsystem m_SwerveSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

        public SnapHome(SwerveSubsystem subsystem ,DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier){
            this.m_SwerveSubsystem = subsystem;
            this.m_translationXSupplier = translationXSupplier;
            this.m_translationYSupplier = translationYSupplier;
            addRequirements(subsystem);
        }
        @Override
        public void execute() {
          m_SwerveSubsystem.dt.driveSnap(-m_translationXSupplier.getAsDouble()*DriveConstants.MAX_FWD_REV_SPEED_MPS, -m_translationYSupplier.getAsDouble()*DriveConstants.MAX_STRAFE_SPEED_MPS, Rotation2d.fromDegrees(180));
      
        }
      
        @Override
        public void end(boolean interrupted) {
        
      
        }

  
}
