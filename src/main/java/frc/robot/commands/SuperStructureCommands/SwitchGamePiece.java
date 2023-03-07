package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class SwitchGamePiece extends CommandBase{
    private final SuperStructure m_superStructure;
    private final boolean gamePieceType;
    
    public SwitchGamePiece(SuperStructure superStructure, boolean type){
        this.m_superStructure = superStructure;
        this.gamePieceType = type;
    }
    @Override
    public void initialize(){
        m_superStructure.hasCone(gamePieceType);

    }
    
    @Override
    public void execute(){
        
       
    }
    @Override
    public void end(boolean interrupted){

    }
    @Override 
    public boolean isFinished(){
        return true;
    }

}

