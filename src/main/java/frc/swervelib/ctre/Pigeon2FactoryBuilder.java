package frc.swervelib.ctre;

//import com.ctre.phoenix.sensors.PigeonIMUSimCollection;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANPIDController.AccelStrategy;

import java.util.ArrayList;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.swervelib.Gyroscope;

public class Pigeon2FactoryBuilder {
    //private static PigeonIMUSimCollection pigeonSim;





    public Gyroscope build(WPI_Pigeon2 pigeon) {
        return new GyroscopeImplementation(pigeon);
    }

    private static class GyroscopeImplementation implements Gyroscope {
        private final WPI_Pigeon2 pigeon;
        private double [] xyzDPS = new double[3];
        

        private GyroscopeImplementation(WPI_Pigeon2 pigeon) {
            this.pigeon = pigeon;
            this.pigeon.zeroGyroBiasNow();
       
            
        }
     
        @Override
        public void zeroGyroscope(double angle) {
           this.pigeon.setYaw(angle);
        
            
            
        }
        @Override
        public Boolean getGyroReady() {
            return true;
        }
        @Override 
        public Rotation2d getGyroRoll(){
            return Rotation2d.fromDegrees(pigeon.getRoll());
        }
        @Override 
        public Rotation2d getGyroPitch(){
            return Rotation2d.fromDegrees(pigeon.getPitch());
        }
        



        @Override
        public Rotation2d getGyroHeading() {
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }/*

        
        @Override
        public double readGetAngle(){
            return pigeon.getAngle();
        }
        @Override
        public Rotation2d readGetYaw(){
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
        @Override
        public Rotation2d readFused(){
            return Rotation2d.fromDegrees(pigeon.getYaw());
        }
        

        @Override
        public void zeroGyroscope() {
            pigeon.setYaw(0.0);
        }*/

        @Override
        public void setAngle(double angle) {
            //pigeonSim.setRawHeading(angle);
        }
        @Override
        public Double pitchVelocity(){
           pigeon.getRawGyro(xyzDPS);
           return xyzDPS[0];
        }
        @Override 
        public Double rollVelocity(){
            pigeon.getRawGyro(xyzDPS);
            return xyzDPS[1];
        }
    }
}
