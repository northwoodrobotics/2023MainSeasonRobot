package frc.robot.subsystems.SuperStructure;

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


public class SuperStructure extends SubsystemBase{

    public ControlState controlState;
    public endEffectorState intakeControlState;
    public double presetElevatorHeight;
    public double presetWristAngle;

    public boolean hasGamePiece;
    public boolean intakeStateHasChanged;
    public double timeStateEntered;
    public double adjustedWristAngle;
    public double adjustedElevatorPosition;
    public double lastElevatorPosition;
    public double lastWristAngle;
    private boolean ejectCone;

    // initialize motor objects
    private LoggedFalcon500 elevatorMotor = new LoggedFalcon500(SuperStructureConstants.ElevatorMotorID); 
    


   // private LoggedNeo intakeMotor = new LoggedNeo(SuperStructureConstants.EndEffectorMotorID, true, 30);
    private LoggedFalcon500 wristMotor = new LoggedFalcon500(SuperStructureConstants.WristMotorID);
    
    // initiate Logging Objects
    private LoggedMotorIOInputsAutoLogged elevatorLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedMotorIOInputsAutoLogged wristLog = new LoggedMotorIOInputsAutoLogged();
   

    // initiate Logged outputs
    private LoggedDashboardNumber wristPositionRadians = new LoggedDashboardNumber("WristPosition Radians");
    private LoggedDashboardNumber elevatorPositionRadians= new LoggedDashboardNumber("ElevatorPosition");
    private LoggedDashboardNumber wristTargetPositionRadians = new LoggedDashboardNumber("Targed Wrist Radians");
    private LoggedDashboardBoolean DashboardhasGamePiece = new LoggedDashboardBoolean("Has Game Piece");
    private LoggedDashboardNumber  wantedElevatorPos = new LoggedDashboardNumber("Wanted Elevator Rad");
    private LoggedDashboardBoolean gamePieceType = new LoggedDashboardBoolean("Cone in Intake");


    public SuperStructure(){  
        ejectCone = false;
        hasGamePiece = false;
        intakeStateHasChanged = false;
        presetElevatorHeight = 0.0;
        presetWristAngle = SuperStructurePresets.stowed.getWristAngleRadians();
        
        controlState = ControlState.preset;
        intakeControlState = endEffectorState.holding;
        //wristPositionRadians.setDefault(SuperStructurePresets.stowed.wristAngleRadians);
        elevatorPositionRadians.setDefault(0.0);
        wristTargetPositionRadians.setDefault(0.0);
        
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
       /*  elevatorMotor.configurePID(
            SuperStructureConstants.ElevatorP, 
            SuperStructureConstants.ElevatorI, 
            SuperStructureConstants.ElevatorD, 
            SuperStructureConstants.ElevatorF, 
            1);*/
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
        wristMotor.setEncoder(160.0);

        elevatorMotor.setEncoder(0.0);
        
       
    }
    
    // returns if the intake has a game piece in it
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
    // list of "Control States" that the elevator and wrist can be in. 
    // preset follows tuned values for each scoring position, wrist and height adjust are what they sound like.
    public enum ControlState{
        preset, 
        wristAdjust,
        heightAdjust,
    }
    // list of end Effector States
    public enum endEffectorState{
        holding(SuperStructureConstants.intakeHoldingPercentOutput), cubeEject(-1.0), intaking(1), empty(0.0),
        coneEject(-0.3);
        public double output;
        private endEffectorState(double output){
            this.output = output;

        }
    }
    
        // internally changing end effector state
    public synchronized void setEndEffectorState(endEffectorState newState) {
        if (newState != intakeControlState){
            intakeStateHasChanged = true;
            intakeControlState = newState;
            timeStateEntered = Timer.getFPGATimestamp();
        }

        
    }


  
    // sets control state ot wrist adjust and feeds adjustment into the controller
    public void adjustWristAngle(double adjustmentDemandDegrees){  
        controlState = ControlState.wristAdjust; 
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedWristAngle = (wristMotor.getPosition()+ adjustmentRadians);
    }
    // checks if the wrist is at the correct height
    public boolean isAtTargetHeight(){
        return (Math.abs(presetElevatorHeight-elevatorMotor.getPosition())< 0.2);
    }
    // checks if wrist is at target angle
    public boolean isAtTargetAngle(){
        return (Math.abs(presetWristAngle-wristMotor.getPosition())< 0.2);
    }
    // sets elevator and wrist to preset height and wrist angle
    public void setSuperStructureState(double elevatorPosition, double wristPosition){
        
        this.presetElevatorHeight = elevatorPosition;
        this.presetWristAngle = wristPosition;
        controlState = ControlState.preset;
        return;
    }
    // feeds height into height adjustment.
    public void adjustElevatorPosition(double adjustmentDemandDegrees){
        controlState = ControlState.heightAdjust;
        double adjustmentRadians = Units.degreesToRadians(adjustmentDemandDegrees);
        adjustedElevatorPosition = (elevatorMotor.getPosition()+ adjustmentRadians);

    }
    // accepts a superstrucure preset 
    public Command acceptSuperStructureState(Supplier<SuperStructureState> targetState){
        return runOnce(()->setSuperStructureState(
            targetState.get().getHeightDemand(), 
            targetState.get().getWristAngleRadians()));
    }

    


    // all control for the superstrucure is done periodcally so any ocolations or robot collisions will be corrected without having to go through the command scheduler.
    @Override 
    public void periodic(){
        
        switch (controlState){
            case preset: 
            
            elevatorMotor.setMotionMagicPosition(presetElevatorHeight, 0, 0);
            lastElevatorPosition = presetElevatorHeight;
            wristMotor.setMotionMagicPosition(MathUtil.clamp(presetWristAngle, SuperStructureConstants.intakeMinAngle, SuperStructureConstants.intakeMaxAngle), 0, 0);
            lastWristAngle = presetWristAngle;
            break;
            case wristAdjust:
            elevatorMotor.setMotionMagicPosition(elevatorMotor.getPosition() , 0, 0);
            wristMotor.setMotionMagicPosition(adjustedWristAngle, 0, 0);
            break; 
            case heightAdjust: 
            elevatorMotor.setMotionMagicPosition(adjustedElevatorPosition, 0, 0);
            wristMotor.setMotionMagicPosition(wristMotor.getPosition(), 0, 0);
            break;
        
        }
        

    
        


        
        // log motor data
        elevatorMotor.updateInputs(elevatorLog);
        Logger.getInstance().processInputs("ElevatorMotor", elevatorLog);
        wristMotor.updateInputs(wristLog);
        Logger.getInstance().processInputs("WristLog", wristLog);
       
        
        wantedElevatorPos.set(presetElevatorHeight);
        DashboardhasGamePiece.set(hasGamePiece);
        gamePieceType.set(ejectCone);
        
        

    }



}
