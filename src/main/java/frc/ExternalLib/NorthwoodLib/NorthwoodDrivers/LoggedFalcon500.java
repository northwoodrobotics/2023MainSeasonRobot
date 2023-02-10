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
    
    public LoggedFalcon500(int motorID){
        this.motorID = motorID;
        this.motor = new TalonFX(this.motorID);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = 12.0;
        config.statorCurrLimit.enable = true;
        config.statorCurrLimit.currentLimit = 40;
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        motor.setInverted(false);


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
  public void setVelocity(double velocityRadPerSec, double ffVolts){
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
  public void configurePID(double kP, double kI, double kD, double ff) {
    motor.config_kP(0, kP);
    motor.config_kI(0, kI);
    motor.config_kD(0, kD);
    motor.config_kF(0, ff);
  }



}
