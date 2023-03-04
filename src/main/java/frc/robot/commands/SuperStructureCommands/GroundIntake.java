package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureBase.endEffectorState;

public class GroundIntake extends CommandBase{
    private final SuperStructure m_superStructure;
    public GroundIntake(SuperStructure superStructure){
        this.m_superStructure = superStructure;
    }
    @Override
    public void initialize(){

    }
    
    @Override
    public void execute(){
        m_superStructure.setSuperStructureState(SuperStructurePresets.groundIntake);
        m_superStructure.conformEndEffectorState(endEffectorState.intaking);
       
    }
    @Override
    public void end(boolean interrupted){
        m_superStructure.setSuperStructureState(SuperStructurePresets.stowed);
        
        
    }
    public boolean isFinished(){
        return m_superStructure.hasGamePiece;
    }

}
