package frc.robot.subsystems.SuperStructure;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;

public class SuperStructureBase extends SubsystemBase{
    public ControlState controlState;
    public endEffectorState intakeControlState;
    public SuperStructureState wantedState;
    public boolean hasGamePiece;
    public boolean intakeStateHasChanged;
    public double timeStateEntered;
    public double adjustedWristAngle;
    public double adjustedElevatorPosition;
    public double lastElevatorPosition;
    public double lastWristAngle;


    public SuperStructureBase(){

    }

    public enum ControlState{
        preset, 
        wristAdjust,
        heightAdjust,
    }

    public enum endEffectorState{
        holding(SuperStructureConstants.intakeHoldingPercentOutput), ejecting(-1.0), intaking(1), empty(0.0);
        public double output;
        private endEffectorState(double output){
            this.output = output;

        }
    }
    public boolean hasGamePiece(){
        return hasGamePiece;
    }
    public boolean getIntakeStateChange(){
        return intakeStateHasChanged;
    }
    

    public synchronized void setEndEffectorState(endEffectorState newState) {
        if (newState != intakeControlState){
            intakeStateHasChanged = true;
            intakeControlState = newState;
            timeStateEntered = Timer.getFPGATimestamp();
        }

        
    }

    public void conformEndEffectorState(endEffectorState targetState){
        
        setEndEffectorState(targetState);
    }

  

    public void adjustWristAngle(double adjustmentDemandDegrees){  
        controlState = ControlState.wristAdjust; 
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedWristAngle = (lastWristAngle+ adjustmentRadians);
    }
    public void setSuperStructureState(SuperStructureState targetState){
        controlState = ControlState.preset;
        wantedState = targetState;
    }
    public void adjustElevatorPosition(double adjustmentDemandDegrees){
        controlState = ControlState.heightAdjust;
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedElevatorPosition = (lastElevatorPosition+ adjustmentRadians);

    }
    



    @Override 
    public void periodic(){

        switch (controlState){
            case preset: 
            
            lastElevatorPosition = wantedState.getHeightDemand();
            
            lastWristAngle = wantedState.getWristAngleRadians();
            break;
            case wristAdjust:
            
            break; 
            case heightAdjust: 

            break;
        }

        switch (intakeControlState){
            case ejecting: 
            if(intakeStateHasChanged){
                hasGamePiece = false;
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2){
                    setEndEffectorState(endEffectorState.empty);
                }
                
            }
            break;
            case intaking:


            
            break;
            case holding:
            break;
            case empty:
            break; 
            
        }

    
}

}
