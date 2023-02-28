package frc.robot.commands.ActionCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class CCRP extends CommandBase{

    private final SwerveSubsystem m_SwerveSubsystem;
    private final SuperStructure m_SuperStructure;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;

        public CCRP(SwerveSubsystem subsystem ,DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, SuperStructure structure){
            this.m_SwerveSubsystem = subsystem;
            this.m_translationXSupplier = translationXSupplier;
            this.m_translationYSupplier = translationYSupplier;
            this.m_SuperStructure = structure;
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
