package frc.robot.commands.ActionCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.swervelib.SwerveSubsystem;

public class AutoBalance extends CommandBase{
  private final SwerveSubsystem m_SwerveSubsystem;
  private ProfiledPIDController  balanceController; 



  public AutoBalance(SwerveSubsystem subsystem) {
    this.m_SwerveSubsystem = subsystem;
    balanceController = new ProfiledPIDController(AutoConstants.BalanceP, AutoConstants.BalanceI, AutoConstants.BalanceD, AutoConstants.BalanceConstraints); 
    balanceController.disableContinuousInput();
    
    

    addRequirements(subsystem);

  }

  @Override
  public void execute() {
    double demand=balanceController.calculate(m_SwerveSubsystem.dt.GyroRoll(), new State(0, 0));
    m_SwerveSubsystem.dt.driveClean(demand, 0, 0);

  }

  @Override
  public void end(boolean interrupted) {
         m_SwerveSubsystem.dt.driveClean(0, 0, 0);

  }
  @Override 
  public boolean isFinished(){
    return Math.abs(m_SwerveSubsystem.dt.GyroRoll()) < AutoConstants.BalanceTolerableAngle;
  }

}

    

