package frc.swervelib.ctre;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.DriveController;
import frc.swervelib.DriveControllerFactory;
import frc.swervelib.EnclosedSteerController;
import frc.swervelib.ModuleConfiguration;
import frc.swervelib.SteerController;
import frc.swervelib.SteerControllerFactory;
import frc.wpiClasses.SimpleMotorWithMassModel;

public final class Falcon500SteerSim{
    private double gearRatio  =  (14.0 / 50.0) * (10.0 / 60.0); 
    private SimpleMotorWithMassModel falconSim = new SimpleMotorWithMassModel(DCMotor.getFalcon500(1), gearRatio, 0.025);
    
    public Falcon500SteerSim createSteerSim(){
        return this;
    }
    public SteerControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }
    private class FactoryImplementation implements SteerControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer steerConfiguration, ModuleConfiguration moduleConfiguration){
            
            return new ControllerImplementation(falconSim);

        }
    }
    private class ControllerImplementation implements SteerController {
        private final SimpleMotorWithMassModel motor;
        private double referenceAngleRadians = 0.0;

        
        //private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(SimpleMotorWithMassModel motor) {
            this.motor = motor;

        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians, double omegaRadiansPerSecond) {
            double currentAngleRadians = Units.rotationsToRadians(motor.getMechanismPositionRev());
            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            

            
            this.referenceAngleRadians = referenceAngleRadians;
        }
        @Override
        public void setSteerEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // TalonFX wants steps for postion.  Steps per 100ms for velocity.  Falcon integrated encoder has 2048 CPR.
            motor.m_curDisplacementRev = position;
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            
        }



        @Override
        public double getStateAngle() {
            double motorAngleRadians = Units.rotationsToRadians(motor.getMechanismPositionRev());
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        @Override
        public DCMotor getSteerMotor() {
            return DCMotor.getFalcon500(1);
        }

        @Override
        public AbsoluteEncoder getAbsoluteEncoder() {
            return null;
        }

        @Override
        public double getOutputVoltage() {
            return motor.m_fwSim.getOutput(0);
        }
        @Override 
        public double getCurrentAmps(){
            return motor.m_fwSim.getCurrentDrawAmps();
              }
        @Override 
        public double getDriveTempCelcius(){
            return 0.0;
        }
        @Override 
        public double getAbsoluteAngle(){
            return  Units.rotationsToRadians(motor.getMechanismPositionRev());
        }


    }
}