package frc.robot.commands.AutoCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SuperStructureCommands.EjectOverride;
import frc.robot.commands.SuperStructureCommands.GroundIntake;
import frc.robot.commands.SuperStructureCommands.HighCone;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.commands.ActionCommands.AutoBalance;
import frc.robot.commands.SuperStructureCommands.SmartEject;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.ReturnToStowed;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class OnePlusHalfBalance extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final PathPlannerTrajectory path = PathPlanner.loadPath("OnePlusHalfBalance", 2, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();
    
    

    public OnePlusHalfBalance(SwerveSubsystem swerve, SuperStructure structure, EndEffector effector){
        eventMap.put("IntakeMode1", new GroundIntake(structure ,effector));
        eventMap.put("Stow", new ReturnToStowed(structure));
      


        FollowPathWithEvents firstCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(path, swerve), path.getMarkers(), eventMap);
       
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(path.getInitialState())),
        structure.acceptSuperStructureState(()-> SuperStructurePresets.midCone),
        new WaitCommand(1.4),
        new EjectOverride(effector, true),
        //new WaitCommand(0.45),
        structure.acceptSuperStructureState(()-> SuperStructurePresets.stowed)
        );
        
    }
}
