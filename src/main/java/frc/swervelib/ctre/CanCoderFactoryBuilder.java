package frc.swervelib.ctre;

import java.util.Optional;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.Timer;
import frc.swervelib.AbsoluteEncoder;
import frc.swervelib.AbsoluteEncoderFactory;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
    private static double angle = 0;
    private Optional<String> canivoreName = Optional.empty();
    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }
    public CanCoderFactoryBuilder withCanivore(Optional<String> canivoreName) {
        this.canivoreName = canivoreName;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;

        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;
            config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

         WPI_CANCoder encoder = new WPI_CANCoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final WPI_CANCoder encoder;

        private EncoderImplementation(WPI_CANCoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngleRetry() {
            double time = Timer.getFPGATimestamp();
			boolean success = false;
			boolean timeout = false;
			do {
				angle = getAbsoluteAngle();
				success = encoder.getLastError() == ErrorCode.OK;
				timeout = Timer.getFPGATimestamp() - time > 2;
			} while (!success && !timeout);

            return angle;
        }

        @Override
        public double getAbsoluteAngle() {
            angle = Math.toRadians(encoder.getPosition());

            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public void setAbsoluteEncoder(double position, double velocity) {
            // Position is in revolutions.  Velocity is in RPM
            // CANCoder wants steps for postion.  Steps per 100ms for velocity.  CANCoder has 4096 CPR.
            encoder.getSimCollection().setRawPosition((int) (position * 4096));
            // Divide by 600 to go from RPM to Rotations per 100ms.  CANCoder has 4096 CPR.
            encoder.getSimCollection().setVelocity((int) (velocity / 600 * 4096));
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
