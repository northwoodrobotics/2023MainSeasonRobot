package frc.robot.commands.TuningCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class ElevatorAdjust extends CommandBase{
    private final SuperStructure m_superStructure;
    private final DoubleSupplier m_heightAdjust;
    
    public ElevatorAdjust(SuperStructure superStructure, DoubleSupplier heightAdjust){
        this.m_superStructure = superStructure;
        this.m_heightAdjust = heightAdjust;
    }
    @Override
    public void initialize(){
        m_superStructure.setSuperStructureState(SuperStructurePresets.tuningPreset);
    }
    
    @Override
    public void execute(){
        m_superStructure.adjustElevatorPosition(SuperStructureConstants.ElevatorAdjustScaler* m_heightAdjust.getAsDouble());
        
       
    }
    @Override
    public void end(boolean interrupted){

    }
    
}
