package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructure.endEffectorState;

public class GroundIntake extends CommandBase{
    private final SuperStructure m_superStructure;
    public GroundIntake(SuperStructure superStructure){
        this.m_superStructure = superStructure;
    }
    @Override
    public void initialize(){
        m_superStructure.setSuperStructureState(SuperStructurePresets.groundIntake.getHeightDemand(), SuperStructurePresets.groundIntake.getWristAngleRadians());
        m_superStructure.conformEndEffectorState(endEffectorState.intaking);
    }
    
    @Override
    public void execute(){
        
       
    }
    @Override
    public void end(boolean interrupted){
        m_superStructure.setSuperStructureState(SuperStructurePresets.stowed.getHeightDemand(), SuperStructurePresets.stowed.getWristAngleRadians());
        m_superStructure.conformEndEffectorState(endEffectorState.holding);
        
        
    }
    @Override
    public boolean isFinished(){
        return m_superStructure.hasGamePiece = true;
    }


}
