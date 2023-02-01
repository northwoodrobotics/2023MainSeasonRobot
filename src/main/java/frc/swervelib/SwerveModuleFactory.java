package frc.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.ExternalLib.GrassHopperLib.BetterSwerveModuleState;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController, namePrefix);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration, String namePrefix) {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration
        );
        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration
        );

        return new ModuleImplementation(driveController, steerContainer,namePrefix);
    }

    private class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;

        
        private ShuffleboardTab tab = Shuffleboard.getTab("SwerveDt");
        private NetworkTableEntry driveVoltageCmdEntry;
        private NetworkTableEntry driveVelocityCmdEntry;
        private NetworkTableEntry steerAngleCmdEntry;

        private ModuleImplementation(DriveController driveController, SteerController steerController, String namePrefix) {
            this.driveController = driveController;
            this.steerController = steerController;

                
    
        }

        @Override
        public void resetWheelEncoder() {
            driveController.resetEncoder();
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public double getSteerAngle() {
            return steerController.getStateAngle();
        }

        @Override
        public ModuleConfiguration getModuleConfiguration() {
            return moduleConfiguration;
        }

        @Override
        public DriveController getDriveController() {
            return driveController;
        }

        @Override
        public SteerController getSteerController() {
            return steerController;
        }

        @Override
        public AbsoluteEncoder getAbsoluteEncoder() {
            return steerController.getAbsoluteEncoder();
        }
        @Override
        public void updateInputs(swerveModuleIOInputs inputs){
            inputs.drivePositionMeters = driveController.getStateMeters();
            inputs.driveVelocity = driveController.getStateVelocity();
            inputs.driveAppliedVolts = driveController.getOutputVoltage();
            inputs.driveCurrentAmps = new double[]{driveController.getCurrentAmps()};
            inputs.driveTempCelcius = new double[]{driveController.getDriveTempCelcius()};

            inputs.steerAbsoluteDeg = steerController.getAbsoluteAngle();
            inputs.steerAngleDeg = steerController.getReferenceAngle();
            inputs.steerAppliedVolts = steerController.getOutputVoltage();
            inputs.steerCurrentAmps = new double[]{steerController.getCurrentAmps()};
            inputs.steerTempCelcius = new double[]{steerController.getDriveTempCelcius()};


        }
        

        
        @Override
        public void set(BetterSwerveModuleState state) {
            double steerAngle = state.angle.getRadians();
            double driveVoltage = state.speedMetersPerSecond/SwerveConstants.MAX_FWD_REV_SPEED_MPS* SwerveConstants.MAX_VOLTAGE;
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle,0 );

                    }
        @Override
        public SwerveModulePosition getPosition(){
            return new SwerveModulePosition(driveController.getStateMeters(), Rotation2d.fromRadians(getSteerAngle()));
        }
       
        @Override
        public void setVelocity(BetterSwerveModuleState state) {
            double steerAngle = state.angle.getRadians();
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                state.speedMetersPerSecond *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setVelocity(state.speedMetersPerSecond);
            steerController.setReferenceAngle(steerAngle, state.omegaRadPerSecond* SwerveConstants.ModuleTwist_KV);
            

           
        }
    }
}
