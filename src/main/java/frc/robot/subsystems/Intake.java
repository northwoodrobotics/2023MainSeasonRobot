// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private WPI_TalonSRX topIntake;
  private WPI_TalonSRX bottomIntake;
  public Intake() {
    
   topIntake = new WPI_TalonSRX(Constants.topIntakeMotorID);
   bottomIntake = new WPI_TalonSRX(Constants.bottomIntakeMotorID);
   topIntake.setInverted(true);
   bottomIntake.setInverted(false);

  }
  
	public void move(double pwr) {
		topIntake.set(pwr);
		bottomIntake.set(pwr);
	}
	
	public void stop() {
		topIntake.stopMotor();
		bottomIntake.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
