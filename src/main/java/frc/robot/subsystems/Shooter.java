// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonSRX topShooter;
  private WPI_TalonSRX bottomShooter;
  private GenericHID xbox; 

  public Shooter() {
   topShooter = new WPI_TalonSRX(Constants.topShooterMotorID);
   bottomShooter = new WPI_TalonSRX(Constants.bottomShooterMotorID);
   topShooter.setInverted(false);
   bottomShooter.setInverted(true);
   xbox = RobotContainer.coDriver.getHID();

  }
  
	public void move(double pwr) {
    if (xbox.getRawButton(2)){
      pwr = pwr*0.6;
    }
    else if (xbox.getRawButton(1)){
      pwr=pwr*0.46;
    }
		topShooter.set(pwr);
		bottomShooter.set(pwr);

	}
	
	public void stop() {
		topShooter.stopMotor();
		bottomShooter.stopMotor();
	}
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
