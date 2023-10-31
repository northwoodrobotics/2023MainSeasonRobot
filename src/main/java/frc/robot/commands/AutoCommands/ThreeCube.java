package frc.robot.commands.AutoCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuperStructureCommands.EjectOverride;
import frc.robot.commands.SuperStructureCommands.GroundIntake;
import frc.robot.commands.SuperStructureCommands.HighCone;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SuperStructureCommands.SmartEject;
import frc.robot.commands.SuperStructureCommands.WaitToRecieve;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.MidCone;
import frc.robot.commands.SuperStructureCommands.MidCube;
import frc.robot.commands.SuperStructureCommands.ReturnToStowed;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.EndEffector.endEffectorState;
import frc.swervelib.SwerveSubsystem;
import frc.robot.commands.DriveCommands.DriveTimeCommand;

public class ThreeCube extends SequentialCommandGroup{
    public final List<PathPlannerTrajectory> ThreeCube = PathPlanner.loadPathGroup("3Cube", 2, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();

    
    

    public ThreeCube(SwerveSubsystem swerve, SuperStructure structure, EndEffector effector){
        eventMap.put("IntakeMode1", 

        new SequentialCommandGroup((new ParallelDeadlineGroup(
            new WaitToRecieve(effector), structure.acceptSuperStructureState(()-> SuperStructurePresets.groundIntake),
            new InstantCommand(()->effector.conformEndEffectorState(endEffectorState.intaking)))
        ), new ReturnToStowed(structure)));
        eventMap.put("Stow1", structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed));
        eventMap.put("MidCube",  structure.acceptSuperStructureState(()-> SuperStructurePresets.midCube));
        eventMap.put("IntakeMode2", new GroundIntake(structure,effector));
        eventMap.put("Stow1",  structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(new AutoDrive(swerve, ThreeCube.get(0)), ThreeCube.get(0).getMarkers(), eventMap);
        // FollowPathWithEvents secondCommand = new FollowPathWithEvents(new AutoDrive(swerve, ThreeCube.get(1)), ThreeCube.get(1).getMarkers(), eventMapTwo);
        
        addCommands(
            //new DriveTimeCommand(swerve, -1.5, 250),
            new InstantCommand(()-> swerve.dt.setKnownState(ThreeCube.get(0).getInitialState())),
            structure.acceptSuperStructureState(()-> SuperStructurePresets.midCone),
            new WaitCommand(1.4),
            new EjectOverride(effector, true),
            structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed),
            new DriveTimeCommand(swerve, 1.25, 2250)
            //firstCommand,
            // new WaitCommand(0.3),
            // new EjectOverride(effector, false),
            // secondCommand,
            // new AutoBalance(swerve)
        );
        
          
 
        
        
    }
        


    }

