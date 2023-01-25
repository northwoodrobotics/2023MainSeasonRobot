package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

;

public class MitchieClimber extends SubsystemBase{

    private TalonFX ClimbLeft = new TalonFX(ClimberConstants.ClimbLeftID);
    private TalonFX ClimbRight = new TalonFX(ClimberConstants.ClimbRightID);
    private DoubleSolenoid ClimbPiston = new DoubleSolenoid(31, PneumaticsModuleType.CTREPCM, 0,1);
    private int ClimbStep; 
    private ClimbState m_ClimbState;
    private double setPoint;
    

    public MitchieClimber(){

        ClimbStep = 0;
        TalonFXConfiguration Climb1Config = new TalonFXConfiguration();
        Climb1Config.slot0.kP = ClimberConstants.Climb1P;
        Climb1Config.slot0.kI = ClimberConstants.Climb1I;
        Climb1Config.slot0.kD = ClimberConstants.Climb1D;
        Climb1Config.slot0.kF = ClimberConstants.Climb1F;
        Climb1Config.motionAcceleration = ClimberConstants.Climb1MotionAccel; 
        Climb1Config.motionCruiseVelocity = ClimberConstants.Climb1MotionVelocity;
        Climb1Config.motionCurveStrength = 1; 
        Climb1Config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        Climb1Config.supplyCurrLimit.currentLimit= 30;
        Climb1Config.supplyCurrLimit.enable = true; 
        ClimbLeft.configAllSettings(Climb1Config);
        ClimbLeft.setNeutralMode(NeutralMode.Brake);
        ClimbLeft.setStatusFramePeriod(3, 200);
        ClimbRight.setStatusFramePeriod(3, 200);

        // fix me 
        ClimbLeft.setInverted(false);
        TalonFXConfiguration Climb2Config = new TalonFXConfiguration();
        Climb2Config.slot0.kP = ClimberConstants.Climb2P;
        Climb2Config.slot0.kI = ClimberConstants.Climb2I;
        Climb2Config.slot0.kD = ClimberConstants.Climb2D;
        Climb2Config.slot0.kF = ClimberConstants.Climb2F;
        Climb2Config.motionAcceleration = ClimberConstants.Climb2MotionAccel;
        Climb2Config.motionCruiseVelocity = ClimberConstants.Climb2MotionVelocity;
        Climb2Config.motionCurveStrength = 1;
        Climb2Config.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        Climb2Config.supplyCurrLimit.currentLimit= 30;
        Climb2Config.supplyCurrLimit.enable = true; 
        ClimbRight.configAllSettings(Climb1Config);
        ClimbRight.setNeutralMode(NeutralMode.Brake);

        
        ClimbRight.setInverted(false);
        ClimbRight.setInverted(false);

        ShutTalonUP(ClimbLeft);
        ShutTalonUP(ClimbRight);



    }

    public void ShutTalonUP(TalonFX targetTalon){
        targetTalon.setStatusFramePeriod(4, 251);
        targetTalon.setStatusFramePeriod(10, 251);
        targetTalon.setStatusFramePeriod(12, 251);
        targetTalon.setStatusFramePeriod(13, 251);
        targetTalon.setStatusFramePeriod(21, 251);
        targetTalon.setStatusFramePeriod(4, 251);

    }

    public void ClimbersToPosition(double Position){
        setPoint = Position;
        ClimbLeft.set(ControlMode.MotionMagic,Position);
        ClimbRight.set(ControlMode.MotionMagic,Position);
    }

    
    public boolean IsClimberAtSetpoint(){
       return ClimbLeft.getSelectedSensorPosition() == setPoint&& ClimbRight.getSelectedSensorPosition() == setPoint;

    }


    public void PistonOut(){
        ClimbPiston.set(Value.kForward);
    }
    public void PistonIn(){
        ClimbPiston.set(Value.kReverse);
    }


    @Override 
    public void periodic(){
    
     // State Machine for MitchieClimber

     switch (m_ClimbState){
         case Stowed: 
         ClimbersToPosition(40);
         PistonIn();
         break; 
         case Deploy:
         ClimbersToPosition(30000);
         PistonIn();
         ClimbStep = 1;
         break; 
         case AutoTraverse: 
         if(ClimbStep == 1){
             ClimbersToPosition(500);
             if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
            
         }
         if (ClimbStep == 2){
            ClimbersToPosition(4000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 3){
            PistonOut();
            ClimbStep ++;
         }
         if (ClimbStep == 4){
            ClimbersToPosition(30000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 5){
            PistonIn();
            ClimbStep ++;
         }
         if (ClimbStep == 6){
            ClimbersToPosition(500);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 7){
            ClimbersToPosition(4000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 8){
            PistonOut();
            ClimbStep ++;
         }
         if (ClimbStep == 9){
            ClimbersToPosition(30000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 10){
            PistonIn();
            ClimbStep ++;
         }
         if (ClimbStep == 11){
            ClimbersToPosition(500);
            ClimbStep ++;
         }
         break;
         case AutoHighBar: 
         if (ClimbStep == 1){
            ClimbersToPosition(500);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 2){
            ClimbersToPosition(4000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 3){
            PistonOut();
            ClimbStep ++;
         }
         if (ClimbStep == 4){
            ClimbersToPosition(30000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 5){
            PistonIn();
            ClimbStep ++;
         }
         if (ClimbStep == 6){
            ClimbersToPosition(500);
            ClimbStep ++;
         }
         break;
         case AutoMid: 
         if (ClimbStep == 1){
            ClimbersToPosition(500);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         if (ClimbStep == 2){
            ClimbersToPosition(4000);
            if (IsClimberAtSetpoint()){
               ClimbStep ++;
             }
         }
         break;

         
         
     }



     
   
    }





    private enum ClimbState{
        Stowed,
        Deploy,
        AutoTraverse, 
        Manual, 
        AutoHighBar, 
        AutoMid
        
    }







    




    
}
