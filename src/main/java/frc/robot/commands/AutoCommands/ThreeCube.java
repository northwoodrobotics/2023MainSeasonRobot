package frc.robot.commands.AutoCommands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SuperStructureCommands.GroundIntake;
import frc.robot.commands.SuperStructureCommands.EjectAndReturnToBottom;
import frc.robot.commands.SuperStructureCommands.HighCube;
import frc.robot.commands.SuperStructureCommands.MidCube;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class ThreeCube extends SequentialCommandGroup{
    public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    HashMap<String, Command> eventMap = new HashMap<>();
    

    public ThreeCube(SwerveSubsystem swerve, SuperStructure structure){
        eventMap.put("ElevatorToMax", new HighCube(structure));
        eventMap.put("ConeDrop", new EjectAndReturnToBottom(structure));
        eventMap.put("IntakeDown1", new GroundIntake(structure));
        eventMap.put("HighCube", new HighCube(structure));
        eventMap.put("DropHigh", new EjectAndReturnToBottom(structure));
        eventMap.put("IntakeGround2", new GroundIntake(structure));
        eventMap.put("CubeMid", new MidCube(structure));
        eventMap.put("DropCubeMid", new EjectAndReturnToBottom(structure));
        FollowPathWithEvents command = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(ThreeCube, swerve), ThreeCube.getMarkers(), eventMap);

        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(ThreeCube.getInitialState())),
        command        
        );
        
    }
}
