package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PathHolder;
import frc.robot.commands.DriveCommands.AutoDrive;
import frc.swervelib.SwerveSubsystem;

public class SquiglyPath extends SequentialCommandGroup{
    public SquiglyPath(SwerveSubsystem swerve){
        addCommands(
        new InstantCommand(()-> swerve.dt.setKnownState(PathHolder.SquiglySquare.getInitialState()))    ,
        new AutoDrive(swerve,PathHolder.SquiglySquare ));
    }
}
