package frc.robot.commands.TuningCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.subsystems.SuperStructure.SuperStructure;

public class WristAdjust extends CommandBase{
    private final SuperStructure m_superStructure;
    private final DoubleSupplier m_angleAdjust;
    
    public WristAdjust(SuperStructure superStructure, DoubleSupplier angleAdjust){
        this.m_superStructure = superStructure;
        this.m_angleAdjust = angleAdjust;
    }
    @Override
    public void initialize(){
        
    }
    
    @Override
    public void execute(){
        m_superStructure.adjustWristAngle(SuperStructureConstants.WristAdjustScaler* m_angleAdjust.getAsDouble());
        
       
    }
    @Override
    public void end(boolean interrupted){

    }
}
