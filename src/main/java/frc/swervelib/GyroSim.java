package frc.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class GyroSim {
    public Gyroscope build(){
        return new GyroscopeImplementation();
    }
    private static class GyroscopeImplementation implements Gyroscope {
        private AnalogGyroSim gyro;



        private GyroscopeImplementation() {
                gyro = new AnalogGyroSim(0);
        }

        @Override
        public Rotation2d getGyroHeading() {
            
          return Rotation2d.fromDegrees(gyro.getAngle());
            
        }

        @Override
        public Boolean getGyroReady() {
            return true;
        }

        @Override
        public void zeroGyroscope(double angle) {
         setAngle(angle);
        }
  /**
   * Gets the Current Roll of the Gyro as a Rotation2d object
   * @return The Rotation2d value of the roll.
   */
    @Override
    public Rotation2d getGyroRoll(){
        return new Rotation2d();
    }
    @Override
    public Rotation2d getGyroPitch(){
        return new Rotation2d();
    }
    @Override
        public Double pitchVelocity(){
           
           return 0.0;
        }
        @Override 
        public Double rollVelocity(){
            
            return 0.0;
        }

        @Override
        public void setAngle(double angle) {
            gyro.setAngle(angle);
        }
    }
}
