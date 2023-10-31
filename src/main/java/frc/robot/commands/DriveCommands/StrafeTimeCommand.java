// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.ExternalLib.NorthwoodLib.Math.BetterSwerveModuleState;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.swervelib.SwerveModule;
import frc.swervelib.SwerveSubsystem;

public class StrafeTimeCommand extends CommandBase {
  private SwerveSubsystem m_drivetrainSubsystem;
  private double m_speed;
  private double m_time;
  private Instant m_startTime = null;
  /** Creates a new DriveTimeCommand. */
  public StrafeTimeCommand(SwerveSubsystem drivetrainSubsystem, double speed, double time) {
    m_speed = speed;
    m_time = time;
    m_drivetrainSubsystem = drivetrainSubsystem;
    addRequirements(m_drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    BetterSwerveModuleState frontLeftWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState frontRightWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState backLeftWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState backRightWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState[] stateArray = {frontLeftWheelState, frontRightWheelState, backLeftWheelState, backRightWheelState};
    m_drivetrainSubsystem.dt.setModuleStates(stateArray);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_startTime == null){
      m_startTime = Instant.now();
    }
    if(Duration.between(m_startTime, Instant.now()).toMillis() > 500){
      BetterSwerveModuleState frontLeftWheelState = new BetterSwerveModuleState(m_speed,new Rotation2d(1.57),0);
      BetterSwerveModuleState frontRightWheelState = new BetterSwerveModuleState(m_speed,new Rotation2d(1.57),0);
      BetterSwerveModuleState backLeftWheelState = new BetterSwerveModuleState(m_speed,new Rotation2d(1.57),0);
      BetterSwerveModuleState backRightWheelState = new BetterSwerveModuleState(m_speed,new Rotation2d(1.57),0);
      BetterSwerveModuleState[] stateArray = {frontLeftWheelState, frontRightWheelState, backLeftWheelState, backRightWheelState};
      m_drivetrainSubsystem.dt.setModuleStates(stateArray);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BetterSwerveModuleState frontLeftWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState frontRightWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState backLeftWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState backRightWheelState = new BetterSwerveModuleState(0,new Rotation2d(1.57),0);
    BetterSwerveModuleState[] stateArray = {frontLeftWheelState, frontRightWheelState, backLeftWheelState, backRightWheelState};
    m_drivetrainSubsystem.dt.setModuleStates(stateArray);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Duration.between(m_startTime, Instant.now()).toMillis() > (m_time + 500);
  }
}
