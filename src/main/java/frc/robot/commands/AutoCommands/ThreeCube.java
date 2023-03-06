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
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SuperStructureCommands.EjectAndReturnToBottom;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.MidCube;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class ThreeCube extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final List<PathPlannerTrajectory> ThreeCube = PathPlanner.loadPathGroup("3Cube", 4, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();

    
    

    public ThreeCube(SwerveSubsystem swerve, SuperStructure structure){
        eventMap.put("IntakeDown1", new GroundIntake(structure));
        eventMap.put("HighCube", new HighCube(structure));
        eventMapTwo.put("IntakeGround2", new GroundIntake(structure));
        eventMapTwo.put("CubeMid", new MidCube(structure));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(new AutoDrive(swerve, ThreeCube.get(0)), ThreeCube.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(new AutoDrive(swerve, ThreeCube.get(1)), ThreeCube.get(1).getMarkers(), eventMapTwo);
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(ThreeCube.get(0).getInitialState())),
        new HighCone(structure),
        new EjectAndReturnToBottom(structure),
        firstCommand,
        new EjectAndReturnToBottom(structure),
        secondCommand,
        new EjectAndReturnToBottom(structure)    
 
        );
        
    }
}
