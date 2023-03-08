// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.ExternalLib.GrassHopperLib.SecondOrderKinematics;
import frc.ExternalLib.SpectrumLib.util.Alert;
import frc.ExternalLib.SpectrumLib.util.Alert.AlertType;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal()) {
          if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
            if (!invalidRobotAlertSent) {
              new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                  .set(true);
              invalidRobotAlertSent = true;
            }
            return RobotType.ROBOT_SIMBOT;
          } else {
            return robot;
          }
        } else {
          return robot;
        }
      }

      public static Mode getMode() {
        switch (getRobot()) {
          case ROBOT_2023C:
          case ROBOT_2023P:
            return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    
          case ROBOT_SIMBOT:
            return Mode.SIM;
    
          default:
            return Mode.REAL;
        }
      }

      
  public static enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }
  
  public static enum Mode {
    REAL,
    REPLAY,
    SIM
  }
  public static final boolean tuningMode = true;
  public static boolean invalidRobotAlertSent = false;
  private static final RobotType robot = RobotType.ROBOT_2023C;
  
  public static double loopPeriodSeconds = 0.02;
  public static double MinVoltage = 8.0;
  
  public static boolean disableHAL = false;
  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023C, " /media/sda1/");

    public static final class VisionConstants {
        public static final Transform3d robotToCam = (
            new Transform3d(new Translation3d(
                Units.inchesToMeters(6.675197), // X Translation
                Units.inchesToMeters(10.252), //Y Translation
                Units.inchesToMeters(7.216)), // Z Translation
                new Rotation3d(
                    0, 
                    0,
                    Units.degreesToRadians(0.0) // yaw
                    )
                )
                
            );

    }

    public static final class DriveConstants {

        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double TRACKWIDTH_METERS = Units.inchesToMeters(22.750); 
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double WHEELBASE_METERS = Units.inchesToMeters(22.750); 
        public static final SecondOrderKinematics KINEMATICS = new SecondOrderKinematics(
                new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
                new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));
        public static final double WHEEL_DIAMETER_METERS = 0.10033; // .10033 = ~4 inches
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
        public static final int PIGEON_ID = 28;
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5; 
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 4; 
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 6; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(2.109+180); // FIXME Measure and
                                                                                                    // set front left steer offset
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(356.66+180); // FIXME Measure and set
                                                                                                // front right stee steer offset
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 10; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(126.82+180); // FIXME Measure and set
                                                                                             // back left steer offset
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 15; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 14; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(72.77+180); // FIXME Measure and set
                                                                                             // back right steer offset
        // Drivetrain Performance Mechanical limits
        static public final double MAX_FWD_REV_SPEED_MPS = Units.feetToMeters(8);
        static public final double MAX_STRAFE_SPEED_MPS = Units.feetToMeters(8);
        static public final double MAX_ROTATE_SPEED_RAD_PER_SEC = Units.degreesToRadians(180);
        static public final double MAX_TRANSLATE_ACCEL_MPS2 = MAX_FWD_REV_SPEED_MPS / 0.25; // 0-full time of 0.25
                                                                                            // second
        static public final double MAX_ROTATE_ACCEL_RAD_PER_SEC_2 = MAX_ROTATE_SPEED_RAD_PER_SEC / 0.25; // 0-full time
                                                                                                         // of 0.25
                                                                                                         // second
        public static final double MAX_VOLTAGE = 12.0; // Maximum Voltage sent to the drive motors
       
        // SENSOR CONSTANTS
        // Sensor-related constants - pulled from datasheets for the sensors and gearboxes

        static public final Pose2d DFLT_START_POSE = new Pose2d(1.78, 2.67,
                Rotation2d.fromDegrees(0.0));
        static public final double ROBOT_MASS_kg = Units.lbsToKilograms(30);
        static public final double ROBOT_MOI_KGM2 = 1.0 / 12.0 * ROBOT_MASS_kg * Math.pow((WHEELBASE_METERS * 1.1), 2)
                * 2;
        public static final double MASS_kg = Units.lbsToKilograms(30);
        public static final double MOI_KGM2 = 1.0 / 12.0 * MASS_kg * Math.pow((TRACKWIDTH_METERS * 1.1), 2) * 2;
        // degrees per second
        public static final double Min_Rotation_Deg = 25;

        public static final double minTranslationCommand = Units.feetToMeters(0.5);
        public static final double minRotationCommand = Units.degreesToRadians(0.5);

        public static final Pose2d rampPose = new Pose2d(0, 0, Rotation2d.fromDegrees(270));
    }

    public final static class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double TRAJECTORYXkP = 1;
        public static final double TRAJECTORYXkI = 0;
        public static final double TRAJECTORYXkD = 0;
        public static final double TRAJECTORYYkP = 1  ;
        public static final double TRAJECTORYYkI = 0;
        public static final double TRAJECTORYYkD = 0;
        public static final double DriveKS = 1.1152;
        public static final double DriveKV = 0.62013;
        public static final double DriveKA = 0.12412;
        public static final double THETACONTROLLERkP = 1;
        public static final double THETACONTROLLERkI = 0;
        public static final double THETACONTROLLERkD = 0;
        // FIXME In order for auto to work consistently and as viewed on the path
        // software
        // Tape Measure out 1 Meter and Drive back and forth on it, change this number
        // until odometry says 0-1m.
        public static final double TractionConstant = Units.feetToMeters(13.5);
        public static final TrapezoidProfile.Constraints BalanceConstraints = 
        new Constraints(
          1.0, // velocity in meters per second that the robot is allowed to pursue when balancing 
          1.0 // maximum acceleration the robot can follow
          );

        public static final double BalanceP = 0.01; 
        public static final double BalanceI = 0.00;
        public static final double BalanceD = 0.00;
        public static final SimpleMotorFeedforward BalanceFeedForward = 
        new SimpleMotorFeedforward(DriveKS, DriveKV);
        public static final double BalanceTolerableAngle = 0.0;


        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class SuperStructureConstants{
        public static final int ElevatorMotorID = 1; 
        public static final int WristMotorID = 2; 
        public static final int EndEffectorMotorID = 31
        ; 

        public static final double intakeCurrentSpikeThreashhold = 20; 
        public static final double intakeHoldingPercentOutput = 0.1; 

        public static final double intakeMinAngle = 120.0;
        public static final double intakeMaxAngle = 600.0;

        public static final double MotionProfileElevatorP = 0.07; // FIXME set PIDF constant for Elevator
        public static final double MotionProfileElevatorD = 0.02; // FIXME set PIDF constant for Elevator
        public static final double MotionProfileElevatorI = 0.0; // FIXME set PIDF constant for Elevator
        public static final double MotionProfileElevatorF = 0.1; // FIXME set PIDF constant for Elevator 
        public static final double ElevatorP = 0.5; // FIXME set PIDF constant for Elevator 
        public static final double ElevatorD = 0.0; // FIXME set PIDF constant for Elevator
        public static final double ElevatorI = 0.0; // FIXME set PIDF constant for Elevator 
        public static final double ElevatorF = 0.045; // FIXME set PIDF constant for Elevator
        public static final double ElevatorMotionAccel = 20480; // FIXME set PIDF constant for Elevator
        public static final double ElevatorMotionVelocity =20480;  // FIXME set PIDF constant for Elevator
        public static final int ElevatorCurrentLimit = 30; //FIXME set Current Limit
        public static final double WristP = 0.05; // FIXME set PIDF constant for Wrist
        public static final double WristF = 0.1; // FIXME set PIDF constant for Elevator
        public static final double WristD = 0.2; // FIXME set PIDF constant for Elevator 
        public static final double WristI = 0.0; // FIXME set PIDF constant for Elevator 
        public static final double WristMotionAccel = 8192.0; // FIXME set PIDF constant for Elevator
        public static final double WristMotionVelocity =8192.0;  // FIXME set PIDF constant for Elevator
        public static final int WristCurrentLimit = 30; //FIXME Current Limit


        public static final double ElevatorAdjustScaler = (360*20.0);
        public static final double WristAdjustScaler = (10*20.0);


        public static final class SuperStructurePresets{
            public static final SuperStructureState groundIntake = new SuperStructureState(
                20.9, // FIXME Measure
                376.0  // FIXME Measure
                ); 
            public static final SuperStructureState lowDrop = new SuperStructureState(
                0.0, // FIXME Measure
                0.0 // FIXME Measure
                );
            public static final SuperStructureState midCube = new SuperStructureState(
                0.0, // FIXME Measure
                0.0 // FIXME Measure
                );
            public static final SuperStructureState midCone = new SuperStructureState(
              221.0, // FIXME Measure
                190.0 // FIXME Measure
                );
            public static final SuperStructureState highCone = new SuperStructureState(
                457.0, // FIXME Measure
                245.0 // FIXME Measure
                );
            public static final SuperStructureState highCube = new SuperStructureState(
                392.2, // FIXME Measure
                279.0 // FIXME Measure
                );
            public static final SuperStructureState humanPlayer = new SuperStructureState(
                0.0, // FIXME Measure
                0.0 // FIXME Measure
                );
            public static final SuperStructureState ramp = new SuperStructureState(
                0.0, // FIXME Measure
                0.0 // FIXME Measure
                );
            public static final SuperStructureState stowed  = new SuperStructureState(
                20.9, // FIXME Measure
                160 // FIXME Measure
                );
            public static final SuperStructureState init = new SuperStructureState(
                0.0, // FIXME Measure
                0.0 // FIXME Measure
                );
            public static final SuperStructureState tuningPreset = new SuperStructureState(
                0.0, // FIXME Measure
                Units.degreesToRadians(90.0) // FIXME Measure
                );
                         
        }
      


    }

 


}
