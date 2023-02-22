// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervelib;

import frc.ExternalLib.GrassHopperLib.SecondOrderKinematics;
import frc.ExternalLib.NorthwoodLib.Math.BetterSwerveModuleState;
import frc.wpiClasses.QuadSwerveSim;
import frc.wpiClasses.SwerveModuleSim;
import frc.wpiClasses.simModuleInputsAutoLogged;
import frc.wpiClasses.SwerveModuleSim.simModuleInputs;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends SubsystemBase {
  private BetterSwerveModuleState[] states;
  private SwerveModulePosition[] positions;
  private swerveModuleIOInputsAutoLogged[] inputs = new swerveModuleIOInputsAutoLogged[]{
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(), 
    new swerveModuleIOInputsAutoLogged(),
  };
  private simModuleInputsAutoLogged[] simInputs = new simModuleInputsAutoLogged[]{
    new simModuleInputsAutoLogged(), 
    new simModuleInputsAutoLogged(), 
    new simModuleInputsAutoLogged(), 
    new simModuleInputsAutoLogged(),
  };
  
 
  private ArrayList<SwerveModule> modules = new ArrayList<SwerveModule>(QuadSwerveSim.NUM_MODULES);
  private ArrayList<SwerveModuleSim> simModules = new ArrayList<SwerveModuleSim>(QuadSwerveSim.NUM_MODULES);

  public SwerveDrivetrainModel dt;

  public SwerveSubsystem(SwerveDrivetrainModel dt) {
    this.dt = dt;
    modules = dt.getRealModules();
    simModules = dt.getModules();
  }  

  @Override
  public void periodic() {

    states = dt.getSwerveModuleStates();
    dt.setModulePositions();
    positions = dt.getModulePositions();


   
      

      for (int i = 0; i<4; i++){
        simModules.get(i).updateInputs(simInputs[i]);
        Logger.getInstance().processInputs("DriveModule"+(Integer.toString(i+1)), simInputs[i]);
      }

    

    if (states != null) {


      SecondOrderKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_FWD_REV_SPEED_MPS);

      modules.get(0).set(states[0]);
      modules.get(1).set(states[1]);
      modules.get(2).set(states[2]);
      modules.get(3).set(states[3]);


      dt.m_poseEstimator.update(dt.getGyroscopeRotation(), 
      new BetterSwerveModuleState[]{
        states[0], 
        states[1], 
        states[2], 
        states[3],
      },
      new SwerveModulePosition[]{
        positions[0],
        positions[1],
        positions[2],
        positions[3]  
      }


      );
      

      
      for (int i = 0; i<4; i++){
        modules.get(i).updateInputs(inputs[i]);
        Logger.getInstance().processInputs("DriveModule"+(Integer.toString(i+1)), inputs[i]);
      }
      for (int i = 0; i<4; i++){
        
        Logger.getInstance().recordOutput("SwerveModuleStates", states[i].toSwerveModuleState());
      }
      Logger.getInstance().recordOutput("Pose Estimator", dt.getPose());  
    }

    }


  

  @Override
  public void simulationPeriodic() {
    dt.update(DriverStation.isDisabled(), 13.2);
  }
}
