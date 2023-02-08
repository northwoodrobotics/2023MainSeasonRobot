package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;


public class SuperStructure extends SubsystemBase{
    // Elevator class 
    
    // initialize motor objects
    private CANSparkMax elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.ElevatorMotorID, MotorType.kBrushless);
    private CANSparkMax wristMotor = new CANSparkMax(Constants.ElevatorConstants.WristMotorID, MotorType.kBrushless);
    private CANSparkMax intakeMotor = new CANSparkMax(ElevatorConstants.EndEffectorMotorID, MotorType.kBrushless);

    // initalize PID controllers
    private SparkMaxPIDController elevatorController;
    private SparkMaxPIDController wristController;
    private SparkMaxPIDController intakeController;

    // intialize internal encoders 
    private SparkMaxRelativeEncoder elevatorSensor;
    private SparkMaxRelativeEncoder wristSensor;
    private SparkMaxRelativeEncoder intakeSensor;

    //State Machine Logic Objects: 
    
    private ElevatorControlState controlState;
    private ElevatorPresets wantedPreset;
    private endEffectorState wantedState;
    private endEffectorPosition wantedPosition;


    public SuperStructure(){  
        wantedPosition = endEffectorPosition.STOWED;


        elevatorController = elevatorMotor.getPIDController();
        elevatorController.setFeedbackDevice(elevatorSensor);
        elevatorController.setP(ElevatorConstants.ElevatorP);
        elevatorController.setI(ElevatorConstants.ElevatorI);
        elevatorController.setD(ElevatorConstants.ElevatorD);
        elevatorController.setFF(ElevatorConstants.ElevatorF);
        elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        elevatorController.setSmartMotionMaxAccel(ElevatorConstants.ElevatorMotionAccel, 0);
        elevatorController.setSmartMotionMaxVelocity(ElevatorConstants.ElevatorMotionVelocity, 0);


        wristController = wristMotor.getPIDController();
        wristController.setFeedbackDevice(wristSensor);
        wristController.setP(ElevatorConstants.WristP);
        wristController.setI(ElevatorConstants.WristI);
        wristController.setD(ElevatorConstants.WristD);
        wristController.setFF(ElevatorConstants.WristF);
        wristController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        wristController.setSmartMotionMaxAccel(ElevatorConstants.WristMotionAccel, 0);
        wristController.setSmartMotionMaxVelocity(ElevatorConstants.WristMotionVelocity, 0);

        intakeController = intakeMotor.getPIDController();
        intakeController.setFeedbackDevice(intakeSensor);
        intakeController.setP(ElevatorConstants.WristP);
        intakeController.setI(ElevatorConstants.WristI);
        intakeController.setD(ElevatorConstants.WristD);
        intakeController.setFF(ElevatorConstants.WristF);
         
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
        switch (wantedPosition){
            
            case CLEARED: 
            

        }



        switch (controlState){

            

        }

    }



}
