package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure.EndEffector;

public class EjectOverride extends CommandBase{
    private final EndEffector m_EndEffector;
    

    
    public EjectOverride(EndEffector effector, boolean isCone){
        this.m_EndEffector = effector;
       
    }
    @Override
    public void initialize(){
        m_EndEffector.ejectGamePiece();
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
