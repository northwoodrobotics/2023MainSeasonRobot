package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedFalcon500;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedMotorIOInputsAutoLogged;
import frc.ExternalLib.NorthwoodLib.NorthwoodDrivers.LoggedNeo;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


public class SuperStructure extends SubsystemBase{
    // Elevator class 
    
    // initialize motor objects
    private LoggedFalcon500 elevatorMotor = new LoggedFalcon500(ElevatorConstants.ElevatorMotorID); 
    private LoggedNeo intakeMotor = new LoggedNeo(ElevatorConstants.EndEffectorMotorID);
    private LoggedNeo wristMotor = new LoggedNeo(ElevatorConstants.WristMotorID);

    // initiate Logging Objects
    private LoggedMotorIOInputsAutoLogged elevatorLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedMotorIOInputsAutoLogged wristLog = new LoggedMotorIOInputsAutoLogged();
    private LoggedMotorIOInputsAutoLogged intakeLog = new LoggedMotorIOInputsAutoLogged();

    //State Machine Logic Objects:     
    private ElevatorControlState controlState;
    private ElevatorPresets wantedPreset;
    private endEffectorState wantedState;
    private endEffectorPosition wantedPosition;


    public SuperStructure(){  
        wantedPosition = endEffectorPosition.STOWED;

        //configure Elevator Motion Profile
        /* Motion Profiles: Using "Motion Planning" as our control method is analagous to how one travels from place to place on a car or bike. 
         * When you are close to where you want to be, you pre-emtively slow down, comming to a stop exactly where you intend to. 
         * In order to make our control of a mechanism (in this case an elevator) perfom controlably, predicatbly and smoothly, we use the 
         * Falcon 500's integrated motion planning control mode, called mtion magic. 
          */
        elevatorMotor.configurePID(
            ElevatorConstants.MotionProfileElevatorP, 
            ElevatorConstants.MotionProfileElevatorI, 
            ElevatorConstants.MotionProfileElevatorD, 
            ElevatorConstants.MotionProfileElevatorF, 
            0); /*this is important, the Falcon 500 can retain several PIDF constant slots.
             for example, slot 1 could have [1, 0, 0.05, 0, 0] and slot 2 could have [ 0.5, 0, 0 ,0 ] allowing us to use different control methods.
             in this case, we will use slots 1 and 2, for a "motion profile" and an emergency standard position control method.*/
        elevatorMotor.configureMotionMagic(
            ElevatorConstants.ElevatorMotionVelocity, 
            ElevatorConstants.ElevatorMotionAccel, 
            0); 
        // now we configure the constants for elevator position control:
        elevatorMotor.configurePID(
            ElevatorConstants.ElevatorP, 
            ElevatorConstants.ElevatorI, 
            ElevatorConstants.ElevatorD, 
            ElevatorConstants.ElevatorF, 
            1);
        // congfigure Wrist Motion Profile
        wristMotor.configurePID(
            ElevatorConstants.WristP, 
            ElevatorConstants.WristI, 
            ElevatorConstants.WristD, 
            ElevatorConstants.WristF, 
            0);
        wristMotor.configureSmartMotion(
            ElevatorConstants.WristMotionVelocity, 
            ElevatorConstants.WristMotionAccel, 
            0, 
            0, 
            AccelStrategy.kTrapezoidal);
       


         
    }
    

    
    private enum endEffectorState{
        HOLDING, EJECTING, INTAKING
    }
    private enum ElevatorControlState{
        MovingToPosition, 
        HoldingPosition, 
        OPENLOOP
    }
    private enum ElevatorPresets{
        GROUNDPICKUP, 
        STOWED, 
        CONEMIDDLE, 
        CONEHIGH, 
        CUBEMIDDLE, 
        CUBEHIGH, 
        STATIONPICKUP
    }

    private enum endEffectorPosition{
        CLEARED, STOWED, OBSTRUCTED
    }

    @Override 
    public void periodic(){
        elevatorMotor.updateInputs(elevatorLog);
        Logger.getInstance().processInputs("ElevatorMotor", elevatorLog);
        wristMotor.updateInputs(wristLog);
        Logger.getInstance().processInputs("WristLog", wristLog);
        intakeMotor.updateInputs(intakeLog);
        Logger.getInstance().processInputs("ElevatorMotor", intakeLog);


   

    }



}
