// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleShooter extends CommandBase {
  /** Creates a new TeleShooter. */
  
    
  private Shooter shooter;
  private DoubleSupplier pwr;
  public TeleShooter(Shooter shooter,DoubleSupplier pwr) {
      this.shooter = shooter;
      this.pwr = pwr;
      addRequirements(shooter);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    shooter.move(
      
      MathUtil.applyDeadband( pwr.getAsDouble(), 0.1));
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
