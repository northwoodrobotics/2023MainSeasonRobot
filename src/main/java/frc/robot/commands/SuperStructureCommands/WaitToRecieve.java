package frc.robot.commands.SuperStructureCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructure.endEffectorState;
public class WaitToRecieve extends CommandBase{
private final SuperStructure m_superStructure;
    
public WaitToRecieve(SuperStructure superStructure){
    this.m_superStructure = superStructure;
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
        return m_superStructure.getIntakeStateChange();
    }
}
