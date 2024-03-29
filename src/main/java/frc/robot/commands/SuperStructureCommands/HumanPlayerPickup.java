package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.EndEffector.endEffectorState;
public class HumanPlayerPickup extends CommandBase{
    private final SuperStructure m_superStructure;
    private final EndEffector m_endEffector;
    
    public HumanPlayerPickup(SuperStructure superStructure, EndEffector endEffector){
        this.m_superStructure = superStructure;
        this.m_endEffector = endEffector;
    }
    @Override
    public void initialize(){
        m_endEffector.setEndEffectorState(endEffectorState.intaking);
        m_superStructure.setSuperStructureState(SuperStructurePresets.humanPlayer.getHeightDemand(), SuperStructurePresets.humanPlayer.getWristAngleRadians());

    }
    
    @Override
    public void execute(){
        
       
    }
    @Override
    public void end(boolean interrupted){
        m_superStructure.setSuperStructureState(SuperStructurePresets.humanPlayer.getHeightDemand(), SuperStructurePresets.humanPlayer.getWristAngleRadians());

        
        
    }
    public boolean isFinished(){
        return m_superStructure.getIntakeStateChange();
    }

}
