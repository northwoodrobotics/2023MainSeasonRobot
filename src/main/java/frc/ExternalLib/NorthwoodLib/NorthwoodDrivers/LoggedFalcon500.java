package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;


public class LoggedFalcon500 implements LoggedMotor{
    private static final double TICKS_PER_REV = 2048;
    private final int motorID;
    private final TalonFX motor;
    private final TalonFXConfiguration config;
    
    public LoggedFalcon500(int motorID){
        this.motorID = motorID;
        this.motor = new TalonFX(this.motorID);
        this.config = new TalonFXConfiguration();
        this.config.voltageCompSaturation = 12.0;
        this.config.statorCurrLimit.enable = true;
        this.config.statorCurrLimit.currentLimit = 40;
        this.config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        motor.setInverted(false);
        motor.configAllSettings(this.config);


    }
    @Override
    public void updateInputs(LoggedMotorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(
        motor.getSelectedSensorPosition() / TICKS_PER_REV);
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV);
    inputs.appliedVolts = motor.getMotorOutputVoltage();
    inputs.statorAmps = new double[] { motor.getSupplyCurrent()};
    inputs.statorTempCelcius = new double[]{motor.getTemperature()};
  }
  @Override 
  public void setVelocity(double velocityRadPerSec, double ffVolts, int slotID){
    motor.selectProfileSlot(slotID, 0);
    double velocityFalconUnits = Units.radiansToRotations(velocityRadPerSec)
         * TICKS_PER_REV / 10.0;
    motor.set(ControlMode.Velocity, velocityFalconUnits,
        DemandType.ArbitraryFeedForward, ffVolts / 12.0);
        
  }
  @Override
  public void stop() {
    motor.set(ControlMode.PercentOutput, 0.0);
  }
  @Override
  public void setPosition(double positionRad, int slotID){
    motor.selectProfileSlot(slotID, slotID);
    double positionFalconUnits = Units.radiansToRotations(positionRad)*TICKS_PER_REV;
    motor.set(ControlMode.Position, positionFalconUnits);
  }

  public void setMotionMagicPosition(double positionRad, double ff, int slotID){
    motor.selectProfileSlot(slotID, slotID);
    double positionFalconUnits = Units.radiansToRotations(positionRad)*TICKS_PER_REV;
    motor.set(ControlMode.MotionMagic, positionFalconUnits, DemandType.ArbitraryFeedForward, ff);
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double ff, int slotID) {
    motor.config_kP(slotID, kP);
    motor.config_kI(slotID, kI);
    motor.config_kD(slotID, kD);
    motor.config_kF(slotID, ff);
  }
  public void configureMotionMagic(double maxVelocity, double maxAcceleration, int curveStrength){
    this.config.motionCurveStrength = curveStrength;
    this.config.motionCruiseVelocity = maxVelocity;
    this.config.motionAcceleration = maxAcceleration;
    motor.configAllSettings(config);
    

  }
  @Override
  public double getPosition(){
    return Units.rotationsToRadians(motor.getSelectedSensorPosition() / TICKS_PER_REV);
  }
  @Override
  public double getVelocity(){
    return Units.rotationsPerMinuteToRadiansPerSecond(motor.getSelectedSensorVelocity() * 10 / TICKS_PER_REV);
  }



}
