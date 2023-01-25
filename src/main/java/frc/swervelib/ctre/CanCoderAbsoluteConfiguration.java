package frc.swervelib.ctre;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
  //  private final SensorInitializationStrategy initStrategy;
  public CanCoderAbsoluteConfiguration(int id, double offset) {
    this.id = id;
    this.offset = offset;
}

public int getId() {
    return id;
}

public double getOffset() {
    return offset;
}
}
