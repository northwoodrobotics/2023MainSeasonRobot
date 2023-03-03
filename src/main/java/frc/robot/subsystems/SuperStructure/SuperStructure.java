package frc.robot.subsystems.SuperStructure;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedFalcon500;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedMotorIOInputsAutoLogged;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedNeo;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;;


public class SuperStructure extends SuperStructureBase{



    // initialize motor objects
    private LoggedFalcon500 elevatorMotor = new LoggedFalcon500(SuperStructureConstants.ElevatorMotorID); 
    private double elevatorEncoderPositionCoefficient =(1.0/12.0);
    


    private CANSparkMax intakeMotor = new CANSparkMax(SuperStructureConstants.EndEffectorMotorID, MotorType.kBrushless );
    private LoggedFalcon500 wristMotor = new LoggedFalcon500(SuperStructureConstants.WristMotorID);
    private double wristEncoderPositionCoefficient = (1/64* 12/24);

    // initiate Logging Objects
    private LoggedMotorIOInputsAutoLogged elevatorLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedMotorIOInputsAutoLogged wristLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedMotorIOInputsAutoLogged intakeLog = new LoggedMotorIOInputsAutoLogged();




    public SuperStructure(){  
        
        hasGamePiece = true;
        intakeStateHasChanged = false;
        wantedState = SuperStructurePresets.stowed;
        controlState = ControlState.preset;
        intakeControlState = endEffectorState.holding;

        elevatorMotor.setEncoder(0.0);
        wristMotor.setEncoder(Units.degreesToRadians(90.0)/wristEncoderPositionCoefficient);
        /* Motion Profiles: Using "Motion Planning" as our control method is analagous to how one travels from place to place on a car or bike. 
         * When you are close to where you want to be, you pre-emtively slow down, comming to a stop exactly where you intend to. 
         * In order to make our control of a mechanism (in this case an elevator) perfom controlably, predicatbly and smoothly, we use the 
         * Falcon 500's integrated motion planning control mode, called motion magic. 
          */
        elevatorMotor.configurePID(
            SuperStructureConstants.MotionProfileElevatorP, 
            SuperStructureConstants.MotionProfileElevatorI, 
            SuperStructureConstants.MotionProfileElevatorD, 
            SuperStructureConstants.MotionProfileElevatorF, 
            0); /*this is important, the Falcon 500 can retain several PIDF constant slots.
             for example, slot 1 could have [1, 0, 0.05, 0, 0] and slot 2 could have [ 0.5, 0, 0 ,0 ] allowing us to use different control methods.
             in this case, we will use slots 1 and 2, for a "motion profile" and an emergency standard position control method.*/
        elevatorMotor.configureMotionMagic(
            SuperStructureConstants.ElevatorMotionVelocity, 
            SuperStructureConstants.ElevatorMotionAccel, 
            0); 

        // configs current limit in Amps
        elevatorMotor.configCurrentLimit(SuperStructureConstants.ElevatorCurrentLimit);

        // now we configure the constants for elevator position control:
        elevatorMotor.configurePID(
            SuperStructureConstants.ElevatorP, 
            SuperStructureConstants.ElevatorI, 
            SuperStructureConstants.ElevatorD, 
            SuperStructureConstants.ElevatorF, 
            1);
        // congfigure Wrist Motion Profile
        wristMotor.configurePID(
            SuperStructureConstants.WristP, 
            SuperStructureConstants.WristI, 
            SuperStructureConstants.WristD, 
            SuperStructureConstants.WristF, 
            0);
        wristMotor.configureMotionMagic(
            SuperStructureConstants.WristMotionVelocity, 
            SuperStructureConstants.WristMotionAccel, 
            0);

        wristMotor.configCurrentLimit(SuperStructureConstants.WristCurrentLimit);
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
        intakeMotor.set(targetState.output);
        setEndEffectorState(targetState);
    }


  

    public void adjustWristAngle(double adjustmentDemandDegrees){  
        controlState = ControlState.wristAdjust; 
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedWristAngle = (wristMotor.getPosition()+ adjustmentRadians);
    }
    public void setSuperStructureState(SuperStructureState targetState){
        controlState = ControlState.preset;
        wantedState = targetState;
    }
    public void adjustElevatorPosition(double adjustmentDemandDegrees){
        controlState = ControlState.heightAdjust;
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedElevatorPosition = (elevatorMotor.getPosition()+ adjustmentRadians);

    }

    



    @Override 
    public void periodic(){
        
        switch (controlState){
            case preset: 
            elevatorMotor.setMotionMagicPosition(wantedState.getHeightDemand()* elevatorEncoderPositionCoefficient, 0, 0);
            lastElevatorPosition = wantedState.getHeightDemand();
            wristMotor.setMotionMagicPosition(wantedState.getWristAngleRadians()* wristEncoderPositionCoefficient, 0, 0);
            lastWristAngle = wantedState.getWristAngleRadians();
            break;
            case wristAdjust:
            elevatorMotor.setMotionMagicPosition(lastElevatorPosition* elevatorEncoderPositionCoefficient, 0, 0);
            wristMotor.setMotionMagicPosition(adjustedWristAngle, 0, 0);
            break; 
            case heightAdjust: 
            elevatorMotor.setMotionMagicPosition(adjustedElevatorPosition* elevatorEncoderPositionCoefficient, 0, 0);
            wristMotor.setMotionMagicPosition(lastWristAngle* wristEncoderPositionCoefficient, 0, 0);
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
            if (intakeMotor.getOutputCurrent() > SuperStructureConstants.intakeCurrentSpikeThreashhold){
                hasGamePiece = true;
                if((Timer.getFPGATimestamp() - timeStateEntered)>0.2){
                    setEndEffectorState(endEffectorState.holding);
                }

            }else 
            
            break;
            case holding:
            break;
            case empty:
            break; 
            
        }

        


        

        elevatorMotor.updateInputs(elevatorLog);
        Logger.getInstance().processInputs("ElevatorMotor", elevatorLog);
        wristMotor.updateInputs(wristLog);
        Logger.getInstance().processInputs("WristLog", wristLog);
        //wristAngleDegrees.set(Units.radiansToDegrees(wristMotor.getPosition()*(1/64* 12/24)) );
        
   

    }



}
