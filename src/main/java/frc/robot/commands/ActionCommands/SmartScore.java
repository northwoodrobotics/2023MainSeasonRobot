package frc.robot.commands.ActionCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ExternalLib.AdvantageLib.GeomUtil;
import frc.robot.Util.FieldConstants;
import frc.robot.subsystems.NodeSelector.ObjectiveTracker.Objective;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.swervelib.SwerveSubsystem;

public class SmartScore extends SequentialCommandGroup{
    
    public SmartScore(SwerveSubsystem drive, 
    SuperStructure structure, 
    Objective objective,
    Command driveCommand, 
    Supplier<Boolean> ejectGamePiece,
    Supplier<Boolean> driveAdjust, 
    Supplier<Boolean> reachScoreDisable
    ){

    }

      /** Returns the position of the target node. */
  public static Translation3d getNodeTranslation(Objective objective) {
    switch (objective.nodeLevel) {
      case HYBRID:
        return FieldConstants.Grids.complexLow3dTranslations[objective.nodeRow];
      case MID:
        return FieldConstants.Grids.mid3dTranslations[objective.nodeRow];
      case HIGH:
        return FieldConstants.Grids.high3dTranslations[objective.nodeRow];
      default:
        return new Translation3d();
    }
}

    public static Pose2d getDriveTarget(
      Pose2d unflippedPose, Objective objective) {
    var pose = unflippedPose;
    var nodeTranslation = getNodeTranslation(objective);
    return new Pose2d();
//
    // Select max arm extension


    // Calculate drive distance
   

    // Increase distance if below min x


    // Get drive pose
    /* 
    var driveTranslation =
        new Pose2d(GeomUtil.translation3dTo2dXY(nodeTranslation), angleFromNode)
            .transformBy(GeomUtil.translationToTransform(distanceFromNode, 0.0))
            .getTranslation();
    driveTranslation =
        new Translation2d(
            driveTranslation.getX(), MathUtil.clamp(driveTranslation.getY(), minDriveY, maxDriveY));
    var driveRotation =
        GeomUtil.translation3dTo2dXY(nodeTranslation)
            .minus(driveTranslation)
            .getAngle()
            .plus(Rotation2d.fromDegrees(shouldScoreFront(pose, objective) ? 0.0 : 180.0));
    return new Pose2d(driveTranslation, driveRotation);
    */
  }
  

  



}
