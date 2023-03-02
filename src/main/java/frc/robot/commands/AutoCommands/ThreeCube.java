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
import frc.robot.commands.SuperStructureCommands.EjectAndReturnToBottom;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.MidCube;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class ThreeCube extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final List<PathPlannerTrajectory> ThreeCube = PathPlanner.loadPathGroup("3 Cube", 4, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();
    HashMap<String, Command> eventMapThree = new HashMap<>();
    
    

    public ThreeCube(SwerveSubsystem swerve, SuperStructure structure){
        eventMap.put("ElevatorToMax", new HighCone(structure));
        eventMapTwo.put("IntakeDown1", new GroundIntake(structure));
        eventMapTwo.put("HighCube", new HighCube(structure));
        eventMapThree.put("IntakeGround2", new GroundIntake(structure));
        eventMapThree.put("CubeMid", new MidCube(structure));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube.get(0), swerve), ThreeCube.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube.get(1), swerve), ThreeCube.get(1).getMarkers(), eventMapTwo);
        FollowPathWithEvents thirdCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube.get(2), swerve), ThreeCube.get(2).getMarkers(), eventMapThree);
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(ThreeCube.get(0).getInitialState())),
        firstCommand,
        new EjectAndReturnToBottom(structure),
        secondCommand,
        new EjectAndReturnToBottom(structure),
        thirdCommand,
        new EjectAndReturnToBottom(structure)        
        );
        
    }
}
