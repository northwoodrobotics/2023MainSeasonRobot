package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class ReturnToStowed extends CommandBase{
    private final SuperStructure m_superStructure;
    
    public ReturnToStowed(SuperStructure superStructure){
        this.m_superStructure = superStructure;
        addRequirements(superStructure);
    }
    @Override
    public void initialize(){
       
    }
    
    @Override
    public void execute(){
        m_superStructure.setSuperStructureState(SuperStructurePresets.stowed);
       
    }
    @Override
    public void end(boolean interrupted){

    }

}
