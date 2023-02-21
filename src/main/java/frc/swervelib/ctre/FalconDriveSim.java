package frc.swervelib.ctre;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.swervelib.DriveController;
import frc.swervelib.DriveControllerFactory;
import frc.swervelib.ModuleConfiguration;
import frc.swervelib.rev.NeoSteerControllerFactoryBuilder.ControllerImplementation;

public final class FalconDriveSim {

   

    private double gearRatio; 


    private FlywheelSim falconSim = new FlywheelSim(DCMotor.getFalcon500(1), gearRatio, 0.025);

    public FalconDriveSim createSim(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }
    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            

            double sensorPositionCoefficient = 2*Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            double sensorVelocityCoefficient = sensorPositionCoefficient * 60;



    
            

            return new ControllerImplementation(falconSim, sensorVelocityCoefficient, sensorPositionCoefficient);
        }


    }

    private class ControllerImplementation implements DriveController {
        private final FlywheelSim motor;
        private double driveMeters = 0.0;
        private double driveAppliedVolts = 0.0;
        
        //private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(FlywheelSim motor, double sensorVelocityCoefficient, double sensorPositionCoefficient) {
            this.motor = motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
           driveAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
           this.motor.setInputVoltage(driveAppliedVolts);
        } 
        @Override
        public void setVelocity(double velocity) {
            // TODO Auto-generated method stub
            
        }

        public void updateSim(){
            double angleDiffRad = motor.getAngularVelocityRadPerSec() * 0.02;
            double angleToMeter = Units.radiansToRotations(angleDiffRad);
            driveMeters += angleToMeter;
        }


        @Override 
        public double getStateMeters(){
            return driveMeters;
        }

        @Override
        public void setDriveEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // TalonFX wants steps for postion.  Steps per 100ms for velocity.  Falcon integrated encoder has 2048 CPR.
            driveMeters = position;
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            double driveAppliedVolts = velocity/6380;
            motor.setInputVoltage(driveAppliedVolts);
        }

        @Override
        public void resetEncoder() {
            driveMeters = 0.0;
        }

        @Override
        public DCMotor getDriveMotor() {
            return DCMotor.getFalcon500(1);
        }

        @Override
        public double getStateVelocity() {
            return motor.getAngularVelocityRadPerSec() ;
        }

        @Override
        public double getOutputVoltage() {
            return driveAppliedVolts;
        }
        @Override 
        public double getCurrentAmps(){
            return motor.getCurrentDrawAmps();
        }
        @Override 
        public double getDriveTempCelcius(){
            return 0.0;
        }
    }


}
