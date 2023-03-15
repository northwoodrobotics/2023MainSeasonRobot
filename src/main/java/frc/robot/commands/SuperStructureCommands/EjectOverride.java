package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure.EndEffector;

public class EjectOverride extends CommandBase{
    private final EndEffector m_EndEffector;
    private boolean override;

    
    public EjectOverride(EndEffector effector, boolean isCone){
        this.m_EndEffector = effector;
        this.override = isCone;
       
    }
    @Override
    public void initialize(){
        m_EndEffector.ejectOverridePiece(override);
    }
    
    @Override
    public void execute(){
        
       
    }
    @Override
    public void end(boolean interrupted){
        //m_superStructure.setSuperStructureState(SuperStructurePresets.stowed.getHeightDemand(), SuperStructurePresets.stowed.getWristAngleRadians());

        
        
    }
    @Override
    public boolean isFinished(){
        return !(m_EndEffector.hasGamePiece());
    }

}
