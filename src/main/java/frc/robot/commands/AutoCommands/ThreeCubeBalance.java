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
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.robot.commands.SuperStructureCommands.SmartEject;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class ThreeCubeBalance extends SequentialCommandGroup{
    
    public final List<PathPlannerTrajectory> ThreeCube = PathPlanner.loadPathGroup("3 Cube Balance", 4, 4);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();

    
    

    public ThreeCubeBalance(SwerveSubsystem swerve, SuperStructure structure, EndEffector effector){
        eventMap.put("IntakeMode1", new GroundIntake(structure,effector));
        eventMap.put("ElevatorToCube", new HighCube(structure));
        eventMapTwo.put("IntakeMode2", new GroundIntake(structure,effector));
        eventMapTwo.put("ElevatorToCone", new HighCone(structure));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube.get(0), swerve), ThreeCube.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube.get(1), swerve), ThreeCube.get(1).getMarkers(), eventMapTwo);
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(ThreeCube.get(0).getInitialState())),
        new HighCone(structure),
        new SmartEject(effector),
        firstCommand,
        new SmartEject(effector),
        secondCommand,
        new SmartEject(effector),    
        new AutoDrive(swerve, ThreeCube.get(2)),
        new AutoBalance(swerve)

 
        );
        
    }
}
