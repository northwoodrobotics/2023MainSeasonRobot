package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ExternalLib.SpectrumLib.gamepads.SpectrumXbox;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructure.endEffectorState;

public class SmartEject extends CommandBase{
    private final SuperStructure m_superStructure;
    

    
    public SmartEject(SuperStructure superStructure){
        this.m_superStructure = superStructure;
       
    }
    @Override
    public void initialize(){
        m_superStructure.ejectGamePiece();
    }
    
    @Override
    public void execute(){
        
       
    }
    @Override
    public void end(boolean interrupted){
        //m_superStructure.setSuperStructureState(SuperStructurePresets.stowed.getHeightDemand(), SuperStructurePresets.stowed.getWristAngleRadians());

        
        
    }


    
}
