// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {
  private Shooter shooter;
	private double pwr;
	private double durationMillis;
	private long startTime;
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, double pwr, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.shooter = shooter;
		  this.pwr = pwr;
		  durationMillis = seconds * 1000;
		  addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    shooter.move(-pwr);
    System.out.println("Started AutoShooter");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.move(-pwr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > startTime + durationMillis;
  }

  protected void end() {
		System.out.println("Ran " + this.toString() + " for " + (System.currentTimeMillis() - startTime) / 1000.0 + " seconds.");
		if (durationMillis != 0) {
			shooter.stop();
		}
}
}
