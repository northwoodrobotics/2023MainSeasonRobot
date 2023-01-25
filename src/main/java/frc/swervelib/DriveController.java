package frc.swervelib;

import edu.wpi.first.math.system.plant.DCMotor;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    void setVelocity(double velocity);

    void resetEncoder();

    DCMotor getDriveMotor();

    double getStateVelocity();

    double getStateMeters();

    double getOutputVoltage();

    double getCurrentAmps();

    double getDriveTempCelcius();

    void setDriveEncoder(double position, double velocity);
}
