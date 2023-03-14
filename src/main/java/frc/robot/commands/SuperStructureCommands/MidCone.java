package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class MidCone extends CommandBase{
    
    private final SuperStructure m_superStructure;
    
    public MidCone(SuperStructure superStructure){
        this.m_superStructure = superStructure;
    }
    @Override
    public void initialize(){
       m_superStructure.acceptSuperStructureState(()-> SuperStructurePresets.midCone);

    }
    
    @Override
    public void execute(){
               
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return m_superStructure.isAtTargetHeight() && m_superStructure.isAtTargetAngle();
    }

}
