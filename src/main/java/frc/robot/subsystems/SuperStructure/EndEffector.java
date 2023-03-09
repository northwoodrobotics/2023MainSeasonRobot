package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;


import edu.wpi.first.math.MathUtil;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedFalcon500;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedMotorIOInputsAutoLogged;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedNeo;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;;

public class EndEffector extends SubsystemBase{
    public endEffectorState intakeControlState;
    public boolean hasGamePiece;
    public boolean intakeStateHasChanged;
    public double timeStateEntered;
    private boolean ejectCone;
    private LoggedMotorIOInputsAutoLogged intakeLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedNeo intakeMotor = new LoggedNeo(SuperStructureConstants.EndEffectorMotorID, true, 30);

    public EndEffector(){
        ejectCone = false;
        hasGamePiece = false;
        intakeStateHasChanged = false;
        intakeControlState = endEffectorState.holding;
    }

    public boolean hasGamePiece(){
        return hasGamePiece;
    }
    // returns if the intake state machine has swapped states
    public boolean getIntakeStateChange(){
        return intakeStateHasChanged;
    }
    // returns if the intake will eject a cone or cube
    public void hasCone(boolean value){
        ejectCone = value;
    }

    public enum endEffectorState{
        holding(SuperStructureConstants.intakeHoldingPercentOutput), cubeEject(-1.0), intaking(1), empty(0.0),
        coneEject(-0.3);
        public double output;
        private endEffectorState(double output){
            this.output = output;

        }
    }

    public synchronized void setEndEffectorState(endEffectorState newState) {
        if (newState != intakeControlState){
            intakeStateHasChanged = true;
            intakeControlState = newState;
            timeStateEntered = Timer.getFPGATimestamp();
        }

        
    }
    // sets end effector state and give motor new output setting
    public void conformEndEffectorState(endEffectorState targetState){
        intakeMotor.setPercentOutput(targetState.output);
        setEndEffectorState(targetState);
    }
    // smart ejection based on internal backup state state
    public void ejectGamePiece(){
       
        if (ejectCone){
            conformEndEffectorState(endEffectorState.coneEject);
        }else
        conformEndEffectorState(endEffectorState.cubeEject);
    }
    // smart ejection with mode override. Used in Smart ejection.
    public void ejectOverridePiece(boolean coneMode){
       
        if (coneMode){
            conformEndEffectorState(endEffectorState.coneEject);
        }else
        conformEndEffectorState(endEffectorState.cubeEject);
    }
     // all control for the superstrucure is done periodcally so any ocolations or robot collisions will be corrected without having to go through the command scheduler.
     @Override 
     public void periodic(){
        switch (intakeControlState){
            case cubeEject: 
            if(intakeStateHasChanged){
                hasGamePiece = false;
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2){
                    conformEndEffectorState(endEffectorState.empty);
                }
                
            }
            break;
            case coneEject: 
            if(intakeStateHasChanged){
                hasGamePiece = false;
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2){
                    conformEndEffectorState(endEffectorState.empty);
                }
                
            }
            break;
            case intaking:
                hasGamePiece = false;
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2){
                    if (intakeMotor.getCurrentAmps() > SuperStructureConstants.intakeCurrentSpikeThreashhold){
                        hasGamePiece = true;
                        
                            conformEndEffectorState(endEffectorState.holding);
                }
               
                    
    
                
            }else

            
            
            break;
            case holding:
            break;
            case empty:
            break; 
            
        }
        intakeMotor.updateInputs(intakeLog);
        Logger.getInstance().processInputs("IntakeLog", intakeLog);

     } 
}
