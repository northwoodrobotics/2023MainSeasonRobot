// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import frc.ExternalLib.SpectrumLib.gamepads.SpectrumXbox;
import frc.ExternalLib.SpectrumLib.gamepads.mapping.ExpCurve;
import frc.robot.commands.ActionCommands.DriveToTag;
import frc.robot.commands.AutoCommands.SquiglyPath;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
import frc.robot.commands.VisionCommands.GetTagPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PhotonCams;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_SwerveSubsystem;
  public static PhotonCams m_cams;
  public static PhotonCamera camera;

  /**
   * SpectrumXbox(0, 0.1, 0.1); is an xbox controller with baked in buttons,
   * triggers, and other logic already there.
   * within the object parameters, the first one is the port, the second and third
   * parameters are deadzone sizes.
   **/
  public static SpectrumXbox driver = new SpectrumXbox(0, 0.1, 0.2);
  private static ShuffleboardTab master = Shuffleboard.getTab("master");


  /**
   * Exp Curves are exponential curves, think parabolas. the first value controls
   * how agressive the curve is, (mess with this one first)
   * the second is it's vertical offset from zero
   * the third value is how stretched or squeezed it is.
   * The final value is how large the deadzone in the middle will be.
   * 
   */
  public static ExpCurve throttleCurve = new ExpCurve(1.2, 0, 1.0, 0.15);
  public static ExpCurve steeringCurve = new ExpCurve(1.2, 0, -1.0, 0.15);


  /**
   * SlewRateLimiters are simple code classes that limit how quickly a value is allowed to change, thus preventing oversteering.
   * these were critical on the Swerve Minibot, due to the non existent torque it had, preventing fast acceleration. 
   * 
   */
  public static SlewRateLimiter xLimiter = new SlewRateLimiter(1.5);
  public static SlewRateLimiter yLimiter = new SlewRateLimiter(1.5);

  /**
   * This Pose2d Object is the field realtive location of a vision target,
   *  an 36h11 family AprilTag, specifically tag #3 
   */
  public Pose2d TagPose;

  public static PathPlannerTrajectory TargetTrajectory;

 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // create drivetrain from our file, utilizing the libary to do position
    // tracking, path following, and a couple of other tricks.
    dt = DrivetrainSubsystem.createSwerveModel();
    m_SwerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);
    camera = new PhotonCamera("gloworm");
    m_cams = new PhotonCams(camera);
    TagPose = new Pose2d();
    PortForwarder.add(5800, "gloworm.local", 5800);
    

    m_SwerveSubsystem.setDefaultCommand(new TeleopDriveCommand(m_SwerveSubsystem,
        () -> xLimiter.calculate(driver.leftStick.getX()),
        () -> yLimiter.calculate(driver.leftStick.getY()),
        () -> -driver.rightStick.getX() * Constants.DriveConstants.MAX_ROTATE_SPEED_RAD_PER_SEC));

        ShowInputs();

    Logger.getInstance().recordOutput("Pose Estimator", new Pose2d(m_SwerveSubsystem.dt.getPose().getTranslation(), m_SwerveSubsystem.dt.getGyroscopeRotation()));


    // Configure the button bindings
    configureButtonBindings();


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driver.aButton.whileTrue(new CalibrateGyro(m_SwerveSubsystem));
    driver.bButton.whileTrue(new DriveToTag(m_SwerveSubsystem, m_cams));
    driver.xButton.onTrue(new GetTagPose(m_cams));
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SquiglyPath(m_SwerveSubsystem);
  }

  public static double getDriveY() {
    return throttleCurve.calculateMappedVal(driver.leftStick.getY());
  }

  public static double getDriveX() {
    return throttleCurve.calculateMappedVal(driver.leftStick.getX());
  }

  public static double getDriveR() {
    double value = driver.rightStick.getY();
    value = steeringCurve.calculateMappedVal(value);
    return value;
  }
  public void ShowInputs(){
    //master.addNumber("TagID", ()-> m_cams.getTagID());
    master.addNumber("X Command", ()-> -xLimiter.calculate(driver.leftStick.getX())*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("Y Command", () -> -yLimiter.calculate(driver.leftStick.getY()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("X Old Command", ()-> -getDriveX()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS* Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    master.addNumber("Y Old Command", () -> -getDriveY() * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    
    master.addNumber("X Input", ()-> (-driver.leftStick.getX()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS));
    master.addNumber("Y Input", () -> (-driver.leftStick.getY()*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS));
    
    master.addNumber("GyroReading", () -> dt.getGyroscopeRotation().getDegrees());
    

    master.addNumber("PoseX", ()-> m_SwerveSubsystem.dt.getPose().getX());
    master.addNumber("PoseY", ()-> m_SwerveSubsystem.dt.getPose().getY());
    master.addNumber("PoseRotation", ()-> m_SwerveSubsystem.dt.getPose().getRotation().getDegrees());
    //master.addNumber("TagX", ()-> m_cams.getTagLocation(m_SwerveSubsystem.dt.getPose()).getX());
    //master.addNumber("Tagy", ()-> m_cams.getTagLocation(m_SwerveSubsystem.dt.getPose()).getY());
    //master.addBoolean("hasTag", ()->m_cams.hasTargets());
   

    
    
    
    
    
  }
}
