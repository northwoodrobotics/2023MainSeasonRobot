package frc.swervelib;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.ExternalLib.GrassHopperLib.BetterSwerveModuleState;

public interface SwerveModule {
    @AutoLog
    public static class swerveModuleIOInputs{
        public double drivePositionMeters = 0.0; 
        public double driveVelocity = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[]{};
        public double[] driveTempCelcius = new double[]{};

        public double steerAngleDeg = 0.0;
        public double steerAbsoluteDeg = 0.0;
        public double steerVelocity = 0.0; 
        public double steerAppliedVolts = 0.0;
        public double[] steerCurrentAmps = new double[]{};
        public double[] steerTempCelcius = new double[]{};
    }
    double getDriveVelocity();

    double getSteerAngle();

    

    ModuleConfiguration getModuleConfiguration();

    DriveController getDriveController();

    SteerController getSteerController();

    AbsoluteEncoder getAbsoluteEncoder();

    SwerveModulePosition getPosition();

    void resetWheelEncoder();

    void set(BetterSwerveModuleState state);
    void updateInputs(swerveModuleIOInputs inputs);

    void setVelocity(BetterSwerveModuleState state);
}
