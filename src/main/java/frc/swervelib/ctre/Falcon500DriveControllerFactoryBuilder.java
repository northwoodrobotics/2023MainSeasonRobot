package frc.swervelib.ctre;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.swervelib.DriveController;
import frc.swervelib.DriveControllerFactory;
import frc.swervelib.ModuleConfiguration;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    private static final int CAN_TIMEOUT_MS_SIM = 100;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS_SIM = 100;
    
    private Optional<String> canivoreName = Optional.empty();

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }
    public Falcon500DriveControllerFactoryBuilder withCanivore(Optional<String> canivoreName) {
        this.canivoreName = canivoreName;
        return this;
    }
    

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasPidConstants()) {
                motorConfiguration.slot0.kP = proportionalConstant;
                motorConfiguration.slot0.kI = integralConstant;
                motorConfiguration.slot0.kD = derivativeConstant;
            }

            if (hasVoltageCompensation()) {
                motorConfiguration.voltageCompSaturation = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
                motorConfiguration.supplyCurrLimit.enable = true;
            }

            WPI_TalonFX motor;
            if (canivoreName.isPresent()) motor = new WPI_TalonFX(driveConfiguration, canivoreName.get());
            else motor = new WPI_TalonFX(driveConfiguration);
            
            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }
            
            
            motor.setNeutralMode(NeutralMode.Brake);
            Timer.delay(1);
            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            Timer.delay(1.0);
            motor.setSensorPhase(true);

            

            // Reduce CAN status frame rates
            if (RobotBase.isSimulation()) {
                motor.setStatusFramePeriod(
                    StatusFrameEnhanced.Status_1_General,
                    STATUS_FRAME_GENERAL_PERIOD_MS_SIM,
                    CAN_TIMEOUT_MS_SIM
                );
            }
            else {
                motor.setStatusFramePeriod(
                        StatusFrameEnhanced.Status_1_General,
                        STATUS_FRAME_GENERAL_PERIOD_MS,
                        CAN_TIMEOUT_MS
                );
            }

            return new ControllerImplementation(motor, sensorVelocityCoefficient, sensorPositionCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final WPI_TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double sensorPositionCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(WPI_TalonFX motor, double sensorVelocityCoefficient, double sensorPositionCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.sensorPositionCoefficient = sensorPositionCoefficient;
            motor.getSimCollection().setBusVoltage(Constants.DriveConstants.MAX_VOLTAGE);
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
        }

        @Override
        public void setVelocity(double velocity, double feedForward) {
            motor.set(TalonFXControlMode.Velocity, velocity / sensorVelocityCoefficient ,DemandType.ArbitraryFeedForward, feedForward/12.0);
        }
        @Override 
        public double getStateMeters(){
            return motor.getSelectedSensorPosition()*sensorPositionCoefficient;
        }

        @Override
        public void setDriveEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // TalonFX wants steps for postion.  Steps per 100ms for velocity.  Falcon integrated encoder has 2048 CPR.
            motor.getSimCollection().setIntegratedSensorRawPosition((int) (position * 2048));
            // Divide by 600 to go from RPM to Rotations per 100ms.  Multiply by encoder ticks per revolution.
            motor.getSimCollection().setIntegratedSensorVelocity((int) (velocity / 600 * 2048));
        }

        @Override
        public void resetEncoder() {
            motor.setSelectedSensorPosition(0);
        }

        @Override
        public DCMotor getDriveMotor() {
            return DCMotor.getFalcon500(1);
        }

        @Override
        public double getStateVelocity() {
            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }

        @Override
        public double getOutputVoltage() {
            return motor.getMotorOutputVoltage();
        }
        @Override 
        public double getCurrentAmps(){
            return motor.getStatorCurrent();
        }
        @Override 
        public double getDriveTempCelcius(){
            return motor.getTemperature();
        }
    }
}
