package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;

import org.littletonrobotics.junction.AutoLog;

/**
 * This is a class that can be used to log motor Data using Advantagekit, 
 * software written by 6328 Mechanical Advantage. In this project we use the data to validate the performance of a used drivetrain motor. 
 * This class logs these parameters: 
 * @param positions in radians
 * @param velocity in radians/second
 * @param current stator voltage 
 * @param applied Apms 
 * @param Tempurature 
 * these can be read through advantagekit 
 * 
 */
public interface LoggedMotor {
    @AutoLog
    public class LoggedMotorIOInputs{
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts =0.0; 
        public double[] statorAmps = new double[]{}; 
        public double[] statorTempCelcius = new double[]{};

    } 
    public default void updateInputs(LoggedMotorIOInputs inputs) {
    }
  
    /** Run closed loop at the specified velocity. */
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {
    }
  
    /** Stop in open loop. */
    public default void stop() {
    }
  
    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD, double ff) {
    }
  }


    

