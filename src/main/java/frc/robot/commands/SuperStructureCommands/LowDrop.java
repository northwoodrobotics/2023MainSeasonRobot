package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class LowDrop extends CommandBase{
    
    private final SuperStructure m_superStructure;
    
    public LowDrop(SuperStructure superStructure){
        this.m_superStructure = superStructure;
    }
    @Override
    public void initialize(){
        //m_superStructure.setSuperStructureState(SuperStructurePresets.lowDrop);
    }
    
    @Override
    public void execute(){
               
    }
    @Override
    public void end(boolean interrupted){

    }

}
