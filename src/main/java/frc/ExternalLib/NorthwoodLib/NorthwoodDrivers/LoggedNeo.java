package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;



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
    public void setVelocity(double velocityRadPerSec, double ffVolts){
        double velocity = Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec);
        controller.setReference(velocity, ControlType.kVelocity);

    }
    @Override
    public void stop() {
      motor.set(0.0);
    }
    @Override
  public void configurePID(double kP, double kI, double kD, double ff) {
    controller.setP(kP, 0);
    controller.setI(kI, 0);
    controller.setD(kD, 0);
    controller.setFF(ff, 0);
  }


}
