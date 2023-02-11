package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.util.Units;


public class LoggedNeo implements LoggedMotor{
    private final int motorID;
    private final CANSparkMax motor; 
    private final SparkMaxPIDController controller;
    private final RelativeEncoder encoder; 
    public LoggedNeo(int motorID){
        this.motorID = motorID;
        this.motor = new CANSparkMax(this.motorID, MotorType.kBrushless);
        this.controller = this.motor.getPIDController();
        this.encoder = motor.getEncoder();
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);
        controller.setFeedbackDevice(encoder);

    }
    @Override
    public void updateInputs(LoggedMotorIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(
        encoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
        encoder.getVelocity());
    inputs.appliedVolts = motor.getBusVoltage()*motor.getAppliedOutput();
    inputs.statorAmps = new double[] { motor.getOutputCurrent()};
    inputs.statorTempCelcius = new double[]{motor.getMotorTemperature()};
    }
    @Override
    public void setVelocity(double velocityRadPerSec, double ffVolts, int slotID){
        double velocity = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
        controller.setReference(velocity, ControlType.kVelocity, slotID);
    }
    @Override
    public void setPosition(double positionRad, int slotID){
        controller.setReference(positionRad, ControlType.kPosition, slotID);
    }
    @Override
    public void stop() {
      motor.set(0.0);
    }
    public void setSmartMotionPosition(double positionRad, int slotID){
      controller.setReference(positionRad, ControlType.kSmartMotion, slotID);
    }
    public void setSmartVelocity(double velocityRadPerSec, int slotID){
      controller.setReference(velocityRadPerSec, ControlType.kSmartVelocity, slotID);
    }
    public void setPercentOutput(double percent){
      motor.set(percent);
    }
    @Override
  public void configurePID(double kP, double kI, double kD, double ff, int slotID) {
    controller.setP(kP, slotID);
    controller.setI(kI, slotID);
    controller.setD(kD, slotID);
    controller.setFF(ff, slotID);
  }
  public void configureSmartMotion(double maxVelocity, double maxAcceleration, double allowableError, int slotID, AccelStrategy strategy){
    controller.setSmartMotionAccelStrategy(strategy, slotID);
    controller.setSmartMotionAllowedClosedLoopError(allowableError, slotID);
    controller.setSmartMotionMaxAccel(maxAcceleration, slotID);
    controller.setSmartMotionMaxVelocity(maxVelocity, slotID);
    
  }


}
