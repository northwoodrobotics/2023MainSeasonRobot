package frc.robot.commands.ActionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.ExternalLib.AdvantageLib.LoggedTunableNumber;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveSubsystem;

public class AutoBalance extends CommandBase{
  private final SwerveSubsystem m_SwerveSubsystem;
  private static final LoggedTunableNumber speedInchesPerSec =
      new LoggedTunableNumber("AutoBalance/SpeedInchesPerSec", 15.0);
  private static final LoggedTunableNumber positionThresholdDegrees =
      new LoggedTunableNumber("AutoBalance/PositionThresholdDegrees", 3.0);
  private static final LoggedTunableNumber velocityThresholdDegreesPerSec =
      new LoggedTunableNumber("AutoBalance/VelocityThresholdDegreesPerSec", 8.0);
  private double angleDegrees;
  
  public AutoBalance(SwerveSubsystem subsystem) {
    this.m_SwerveSubsystem = subsystem;

    
    

    addRequirements(subsystem);

  }
  @Override
  public void initialize(){
    angleDegrees = Double.POSITIVE_INFINITY;
  }

  @Override
  public void execute() {
    angleDegrees = 
    m_SwerveSubsystem.dt.getGyroscopeRotation().getCos()* m_SwerveSubsystem.dt.getGyroPitch().getDegrees()
    + m_SwerveSubsystem.dt.getGyroscopeRotation().getSin() * m_SwerveSubsystem.dt.getGyroRoll().getDegrees();
    double angleVelocityDegreesPerSec =
    m_SwerveSubsystem.dt.getGyroscopeRotation().getCos() * Units.radiansToDegrees(m_SwerveSubsystem.dt.getPitchVelocity())
            + m_SwerveSubsystem.dt.getGyroscopeRotation().getSin() * Units.radiansToDegrees(m_SwerveSubsystem.dt.getRollVelocity());
    boolean shouldStop =
        (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec.get())
            || (angleDegrees > 0.0
                && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec.get());
    if (shouldStop) {
      m_SwerveSubsystem.dt.driveClean(0, 0, 0);
    } else {
      m_SwerveSubsystem.dt.driveClean(
        Units.inchesToMeters(speedInchesPerSec.get()) * (angleDegrees > 0.0 ? -1.0 : 1.0), 
        0,
        0 
        );

    }
  }

  @Override
  public void end(boolean interrupted) {
         m_SwerveSubsystem.dt.driveClean(0, 0, 0);

  }
  @Override 
  public boolean isFinished(){
    return Math.abs(angleDegrees) < positionThresholdDegrees.get();
  }

}

    

