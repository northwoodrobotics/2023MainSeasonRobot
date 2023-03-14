package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.EndEffector.endEffectorState;
public class WaitToRecieve extends CommandBase{
private final EndEffector m_EndEffector;
    
public WaitToRecieve(EndEffector endEffector){
    this.m_EndEffector = endEffector;
    }
    @Override
    public void initialize(){
       // m_superStructure.setEndEffectorState(endEffectorState.intaking);
    }

    @Override
        public void execute(){
    
   
    }
    @Override
        public void end(boolean interrupted){
  //  m_superStructure.setSuperStructureState(SuperStructurePresets.stowed);
    
    
    }
    @Override
    public boolean isFinished(){
        return m_EndEffector.hasGamePiece();
    }
}
