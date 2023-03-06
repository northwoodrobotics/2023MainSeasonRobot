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
        @Override 
        public Double getGyroRoll(){
            return 0.0;
        }

        @Override
        public void setAngle(double angle) {
            gyro.setAngle(angle);
        }
    }
}
