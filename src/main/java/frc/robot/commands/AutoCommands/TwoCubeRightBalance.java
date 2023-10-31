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

public class TwoCubeRightBalance extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2PlusBalance", 2, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMap2 = new HashMap<>();
    
    

    public TwoCubeRightBalance(SwerveSubsystem swerve, SuperStructure structure, EndEffector effector){
        eventMap.put("IntakeMode", 

        new SequentialCommandGroup((new ParallelDeadlineGroup(
            new WaitToRecieve(effector), structure.acceptSuperStructureState(()-> SuperStructurePresets.groundIntake),
            new InstantCommand(()->effector.conformEndEffectorState(endEffectorState.intaking)))
        ), new ReturnToStowed(structure)));
        eventMap.put("IntakeRetract", structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed));
        eventMap.put("MidCube", structure.acceptSuperStructureState(()-> SuperStructurePresets.midCube));
        eventMap2.put("Stow", structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed));
        eventMap2.put("IntakeMode2", new SequentialCommandGroup((new ParallelDeadlineGroup(
            new WaitToRecieve(effector), structure.acceptSuperStructureState(()-> SuperStructurePresets.groundIntake),
            new InstantCommand(()->effector.conformEndEffectorState(endEffectorState.intaking)))
        ), new ReturnToStowed(structure)));
        

                //eventMapTwo.put("CubeMid",  structure.acceptSuperStructureState(()-> SuperStructurePresets.midCone));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(new AutoDrive(swerve, path.get(0)), path.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(new AutoDrive(swerve, path.get(1)), path.get(1).getMarkers(), eventMap2);
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(path.get(0).getInitialState())),
        structure.acceptSuperStructureState(()-> SuperStructurePresets.midCone),
        new WaitCommand(1.4),
        new EjectOverride(effector, true),
        firstCommand,
        new WaitCommand(0.3),
        new EjectOverride(effector, false),
        secondCommand
       
        //new EjectOverride(effector, true)  
 
        );
        
    }
}