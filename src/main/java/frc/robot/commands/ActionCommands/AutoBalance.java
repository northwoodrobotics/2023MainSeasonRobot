package frc.robot.commands.ActionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.swervelib.SwerveSubsystem;

public class AutoBalance extends CommandBase{
  private final SwerveSubsystem m_SwerveSubsystem;



  public AutoBalance(SwerveSubsystem subsystem) {
    this.m_SwerveSubsystem = subsystem;


    

    addRequirements(subsystem);

  }

  @Override
  public void execute() {
    

  }

  @Override
  public void end(boolean interrupted) {
  

  }

}

    

