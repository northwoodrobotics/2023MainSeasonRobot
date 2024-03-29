// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import frc.robot.subsystems.SuperStructure.EndEffector.endEffectorState;

import org.eclipse.jetty.jndi.local.localContextRoot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import frc.ExternalLib.SpectrumLib.gamepads.SpectrumXbox;
import frc.ExternalLib.SpectrumLib.gamepads.mapping.ExpCurve;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.commands.ActionCommands.DriveToRamp;
import frc.robot.commands.ActionCommands.SmartScore;
import frc.robot.commands.AutoCommands.PlaceAndBalance;
import frc.robot.commands.AutoCommands.ThreeCube;
import frc.robot.commands.AutoCommands.ThreeCubeBalance;
import frc.robot.commands.AutoCommands.TwoCubeRightBalance;
import frc.robot.commands.AutoCommands.OnePlusHalfBalance;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.DriveCommands.TeleShooter;
import frc.robot.commands.DriveCommands.TeleIntake;
import frc.robot.commands.DriveCommands.CalibrateGyro;
import frc.robot.commands.DriveCommands.DriveTimeCommand;
import frc.robot.commands.DriveCommands.FeedForwardCharacterization;
import frc.robot.commands.DriveCommands.TeleopDriveCommand;
import frc.robot.commands.DriveCommands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.SuperStructureCommands.SmartEject;
import frc.robot.commands.SuperStructureCommands.SwitchGamePiece;
import frc.robot.commands.SuperStructureCommands.FlipCone;
import frc.robot.commands.SuperStructureCommands.GroundIntake;
import frc.robot.commands.SuperStructureCommands.HighCone;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.HumanPlayerPickup;
import frc.robot.commands.SuperStructureCommands.LowDrop;
import frc.robot.commands.SuperStructureCommands.MidCone;
import frc.robot.commands.SuperStructureCommands.MidCube;
import frc.robot.commands.SuperStructureCommands.ReturnToStowed;
import frc.robot.commands.SuperStructureCommands.WaitToRecieve;
import frc.robot.commands.TuningCommands.WristAdjust;
import frc.robot.commands.TuningCommands.ElevatorAdjust;

import frc.robot.commands.VisionCommands.AddVisionPose;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PhotonCams;
import frc.robot.subsystems.NodeSelector.NodeSelectorServerIO;
import frc.robot.subsystems.NodeSelector.ObjectiveTracker;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
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
  public static EndEffector m_EndEffector;
  public static PhotonCams m_cams;
  public static PhotonCamera camera;
  public static SuperStructure m_SuperStructure;
  private ObjectiveTracker objectiveTracker;
  public static Shooter shooter;
  public static Intake intake;
  public static final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Routine");

  /**
   * SpectrumXbox(0, 0.1, 0.1); is an xbox controller with baked in buttons,
   * triggers, and other logic already there.
   * within the object parameters, the first one is the port, the second and third
   * parameters are deadzone sizes.
   **/
  public static CommandXboxController driver = new CommandXboxController(0);
  public static CommandXboxController coDriver = new CommandXboxController(1);
  private static ShuffleboardTab master = Shuffleboard.getTab("master");
  private static PathPlannerTrajectory testRight3gamePiece = PathPlanner.loadPath("3 Cube Balance", new PathConstraints(2, 2));



  /**
   * SlewRateLimiters are simple code classes that limit how quickly a value is allowed to change, thus preventing oversteering.
   * these were critical on the Swerve Minibot, due to the non existent torque it had, preventing fast acceleration. 
   * 
   */
  public static SlewRateLimiter xLimiter = new SlewRateLimiter(1.5);
  public static SlewRateLimiter yLimiter = new SlewRateLimiter(1.5);


 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   */
  public RobotContainer() {
    // create drivetrain from our file, utilizing the libary to do position
    // tracking, path following, and a couple of other tricks.
    
  

    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C:
        dt = DrivetrainSubsystem.createSwerveModel();
        m_SuperStructure = new SuperStructure();
        m_cams = new PhotonCams();
        objectiveTracker = new ObjectiveTracker(new NodeSelectorServerIO());
        m_EndEffector = new EndEffector();
        shooter = new Shooter();
        intake = new Intake();
        break;
        case ROBOT_2023P:
        dt = DrivetrainSubsystem.createSwerveModel();
        shooter = new Shooter();
          break;
        case ROBOT_SIMBOT:
          dt = DrivetrainSubsystem.createSimSwerveModel();
          m_cams = new PhotonCams();
          m_SuperStructure = new SuperStructure();
         // objectiveTracker = new ObjectiveTracker(new NodeSelectorServerIO());
         shooter = new Shooter();
          break;
      }
    } 
   
    m_SwerveSubsystem = DrivetrainSubsystem.createSwerveSubsystem(dt);
    //m_SuperStructure = new SuperStructure();
   // PortForwarder.add(5800, "photonvision.local", 5800);
    m_cams.setDefaultCommand(new AddVisionPose(m_cams));
    intake.setDefaultCommand(new TeleIntake(intake,() -> ((coDriver.getLeftTriggerAxis()-coDriver.getRightTriggerAxis()))*0.5));
    shooter.setDefaultCommand(new TeleShooter(shooter,() -> coDriver.getLeftY()));

    m_SwerveSubsystem.setDefaultCommand(new TeleopDriveCommand(m_SwerveSubsystem,
        () -> xLimiter.calculate(driver.getLeftY()),
        () -> yLimiter.calculate(driver.getLeftX()),
        () -> driver.getRightX()));

        ShowInputs();
    //m_SuperStructure.setDefaultCommand(new ReturnToStowed(m_SuperStructure));
    
 

    // Configure the button bindings
    configureButtonBindings();

  
    autoChooser.addDefaultOption("Do Nothing", null);
    //autoChooser.addOption("2.5 Piece", new ThreeCube(m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    //autoChooser.addOption("Mid cone and nothing", new OnePlusHalfBalance(m_SwerveSubsystem, m_SuperStructure,m_EndEffector));
    //autoChooser.addOption("Localization Reset", new InstantCommand(()-> dt.setKnownPose(new Pose2d(0, 0, dt.getGyroscopeRotation()))));
    //autoChooser.addOption("2.5 Plus Balance", new ThreeCubeBalance(m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    //autoChooser.addOption("PlaceAndBalance", new PlaceAndBalance(m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    //autoChooser.addOption("2.5 Right Side", new TwoCubeRightBalance(m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    autoChooser.addOption("Mid cone and mobility (long)", new ThreeCubeBalance(shooter,intake,m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    autoChooser.addOption("Mid cone and Balance", new PlaceAndBalance(shooter,intake, m_SwerveSubsystem, m_SuperStructure, m_EndEffector));
    autoChooser.addOption("Mid cone and mobility (short)", new ThreeCube(shooter,intake,m_SwerveSubsystem, m_SuperStructure, m_EndEffector));

    autoChooser.addOption("Characterize Drivetrain", new FeedForwardCharacterization(m_SwerveSubsystem, 
    true, 
    new FeedForwardCharacterizationData("drive"),
    m_SwerveSubsystem.dt ::characterizeDrivetrain,
    m_SwerveSubsystem :: getCharacterizationVelocity
    
   )
   );

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
    //driver.aButton.whileTrue(new CalibrateGyro(m_SwerveSubsystem));
    //driver.bButton.whileTrue(new SequentialCommandGroup(new DriveToRamp(m_SwerveSubsystem, m_SuperStructure), new WaitToRecieve(m_SuperStructure)));
    driver.leftTrigger().whileTrue(new GroundIntake(m_SuperStructure, m_EndEffector));
    driver.leftTrigger().whileFalse(new ReturnToStowed(m_SuperStructure));
    //driver.a().whileTrue(new ReturnToStowed(m_SuperStructure));
    driver.leftBumper().whileTrue(m_SuperStructure.acceptSuperStructureState(()->SuperStructurePresets.humanPlayer )
    .alongWith(new InstantCommand(()-> m_EndEffector.conformEndEffectorState(endEffectorState.intaking))));
    driver.leftBumper().whileFalse(new ReturnToStowed(m_SuperStructure));
    //driver.y().whileTrue(new InstantCommand(()-> m_SuperStructure.ejectGamePiece()));
    //driver.rightBumper().whileTrue(new MidCone(m_SuperStructure));
    //driver.leftBumper().whileTrue(new HighCone(m_SuperStructure));
    //driver.leftTrigger().whileTrue(new HighCube(m_SuperStructure));
    driver.x().whileTrue(new FlipCone(m_SuperStructure));
    driver.rightTrigger().whileTrue(
      new SmartScore(m_SuperStructure, 
      m_EndEffector,
      objectiveTracker.objective, 
      ()-> driver.y().getAsBoolean())
    );
    driver.rightTrigger().onFalse(
      new ReturnToStowed(m_SuperStructure)
    );



   /* driver.leftBumper.onTrue(new HighCube(m_SuperStructure));
    driver.rightBumper.onTrue(new MidCube(m_SuperStructure));
    driver.rightTriggerButton.onTrue(new MidCone(m_SuperStructure));
    driver.yButton.onTrue(new HumanPlayerPickup(m_SuperStructure));
    driver.xButton.onTrue(new EjectAndReturnToBottom(m_SuperStructure));
     */
    //coDriver.x().onTrue(new SwitchGamePiece(m_SuperStructure, false));
    //coDriver.y().onTrue(new  SwitchGamePiece(m_SuperStructure, true));
    //coDriver.leftTrigger().whileTrue(new TeleShooter(shooter));
    //coDriver.rightTrigger().whileTrue(new InstantCommand((()->m_EndEffector.conformEndEffectorState(endEffectorState.intaking)), m_EndEffector));
  //  coDriver.yButton.whileTrue(new HighCone(m_SuperStructure));
    //coDriver.a().whileTrue(new WristAdjust(m_SuperStructure,()-> coDriver.getLeftY()));
    //coDriver.b().whileTrue(new ElevatorAdjust(m_SuperStructure,()-> coDriver.getRightY()));

   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.get();
  }
  

  
  public void ShowInputs(){
    //master.addNumber("X Command", ()-> -xLimiter.calculate(driver.leftStick.getX())*Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
    //master.addNumber("Y Command", () -> -yLimiter.calculate(driver.leftStick.getY()) * Constants.DriveConstants.MAX_FWD_REV_SPEED_MPS);
  }
}
