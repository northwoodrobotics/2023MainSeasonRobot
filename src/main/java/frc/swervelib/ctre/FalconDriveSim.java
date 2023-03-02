package frc.swervelib.ctre;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.swervelib.DriveController;
import frc.swervelib.DriveControllerFactory;
import frc.swervelib.ModuleConfiguration;
import frc.wpiClasses.MotorGearboxWheelSim;

public final class FalconDriveSim {

   

    private double gearRatio = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0); 


    private MotorGearboxWheelSim falconSim = new MotorGearboxWheelSim(
        DCMotor.getFalcon500(1), 
        gearRatio,
        0.10033,
        0.0 
        );

    public FalconDriveSim createSim(){
        return this;
    }
    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            

            double arcLengthConversion =(moduleConfiguration.getWheelDiameter()/2.0);
           



    
            

            return new ControllerImplementation(falconSim, arcLengthConversion);
        }


    }

    private class ControllerImplementation implements DriveController {
        private final MotorGearboxWheelSim motor;
        private double driveMeters = 0.0;
        private double driveAppliedVolts = 0.0;
        private double radiansToMeters;
        
        //private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(MotorGearboxWheelSim motor, double arcLengthConversion) {
            this.motor = motor;
            this.radiansToMeters = arcLengthConversion;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            
        } 
        @Override
        public void setVelocity(double velocity) {
            
            
        }


        @Override 
        public double getStateMeters(){
            
            
            return motor.getPositionRev()* 0.10033;
        }

        @Override
        public void setDriveEncoder(double position, double velocity) {
            //this.motor.update(Constants.loopPeriodSeconds);
            // Position is in revolutions.  Velocity is in RPM
            // TalonFX wants steps for postion.  Steps per 100ms for velocity.  Falcon integrated encoder has 2048 CPR.
            //driveMeters = position;
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            //double driveAppliedVolts = velocity/6380;
            //motor.setInputVoltage(driveAppliedVolts);
        }

        @Override
        public void resetEncoder() {
           // this.motor.update(Constants.loopPeriodSeconds);
           // driveMeters = 0.0;
        }

        @Override
        public DCMotor getDriveMotor() {
            return DCMotor.getFalcon500(1);
        }

        @Override
        public double getStateVelocity() {
            
      
            return motor.getWheelSpeed_RPM() ;
        }

        @Override
        public double getOutputVoltage() {
            
            return driveAppliedVolts;
        }
        @Override 
        public double getCurrentAmps(){
           
            return 0.0;
        }
        @Override 
        public double getDriveTempCelcius(){
            return 0.0;
        }
    }


}
