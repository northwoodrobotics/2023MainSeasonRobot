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

public class TwoCubeBalance extends SequentialCommandGroup{
    //public final PathPlannerTrajectory ThreeCube = PathPlanner.loadPath("3Cube", new PathConstraints(Units.feetToMeters(3), Units.feetToMeters(3)));
    public final List<PathPlannerTrajectory> TwoCubeBalance = PathPlanner.loadPathGroup("2PlusBalance", 4, 2);
    HashMap<String, Command> eventMap = new HashMap<>();
    HashMap<String, Command> eventMapTwo = new HashMap<>();
    
    

    public TwoCubeBalance(SwerveSubsystem swerve, SuperStructure structure){
        eventMap.put("ElevatorToMax", new HighCone(structure));
        eventMapTwo.put("IntakeMode", new GroundIntake(structure));
        eventMapTwo.put("ElevatorToMax2", new HighCube(structure));
        FollowPathWithEvents firstCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(TwoCubeBalance.get(0), swerve), TwoCubeBalance.get(0).getMarkers(), eventMap);
        FollowPathWithEvents secondCommand = new FollowPathWithEvents(swerve.dt.createCommandForTrajectory(TwoCubeBalance.get(1), swerve), TwoCubeBalance.get(1).getMarkers(), eventMapTwo);
        
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(TwoCubeBalance.get(0).getInitialState())),
        firstCommand,
        new EjectAndReturnToBottom(structure),
        secondCommand,
        new EjectAndReturnToBottom(structure),
        new AutoDrive(swerve, TwoCubeBalance.get(2))     
        );
        
    }
}
