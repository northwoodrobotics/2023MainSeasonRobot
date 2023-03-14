package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.Timer;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedMotorIOInputsAutoLogged;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedNeo;
import frc.robot.Constants.SuperStructureConstants;

public class EndEffector extends SubsystemBase{
    public endEffectorState intakeControlState;
    public boolean hasGamePiece;
    public boolean intakeStateHasChanged;
    public double timeStateEntered;
    private boolean ejectCone;
    private LoggedMotorIOInputsAutoLogged intakeLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedNeo intakeMotor = new LoggedNeo(SuperStructureConstants.EndEffectorMotorID, true, 30);
    private LoggedDashboardBoolean DashboardhasGamePiece = new LoggedDashboardBoolean("Has Game Piece");
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
        holding(SuperStructureConstants.intakeHoldingPercentOutput), cubeEject(-0.5), intaking(1), empty(0.0),
        coneEject(-0.1);
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
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2&& intakeMotor.getCurrentAmps() > SuperStructureConstants.intakeCurrentSpikeThreashhold){
                    
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
        DashboardhasGamePiece.set(hasGamePiece);
     } 
}
