package frc.robot.commands.AutoCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuperStructureCommands.GroundIntake;
import frc.robot.commands.SuperStructureCommands.HighCone;
import frc.robot.commands.ActionCommands.AutoBalance;
import frc.robot.commands.SuperStructureCommands.SmartEject;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class ThreeCubeRightBalance extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final List<PathPlannerTrajectory> ThreeCubeRight = PathPlanner.loadPathGroup("2PlusBalance", 4, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();
    
    

    public ThreeCubeRightBalance(SwerveSubsystem swerve, SuperStructure structure, EndEffector effector){
        eventMap.put("IntakeMode1", new GroundIntake(structure ,effector));
        eventMap.put("ElevatorToCube", new HighCube(structure));
        eventMapTwo.put("IntakeMode2", new GroundIntake(structure, effector));
        eventMapTwo.put("ElevatorToCone", new HighCone(structure));


        FollowPathWithEvents firstCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCubeRight.get(0), swerve), ThreeCubeRight.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCubeRight.get(1), swerve), ThreeCubeRight.get(1).getMarkers(), eventMapTwo);
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(ThreeCubeRight.get(0).getInitialState())),
        new HighCone(structure),
        new SmartEject(effector),
        firstCommand,   
        new SmartEject(effector),
        secondCommand,
        new SmartEject(effector),
        swerve.dt.createCommandForTrajectory(ThreeCubeRight.get(2), swerve),
        new AutoBalance(swerve)
        );
        
    }
}
