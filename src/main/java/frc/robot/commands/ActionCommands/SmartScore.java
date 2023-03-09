package frc.robot.commands.ActionCommands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.ExternalLib.AdvantageLib.GeomUtil;
import frc.robot.Constants.SuperStructureConstants.SuperStructurePresets;
import frc.robot.Util.FieldConstants;

import frc.robot.subsystems.NodeSelector.ObjectiveTracker.Objective;
import frc.robot.subsystems.SuperStructure.EndEffector;
import frc.robot.subsystems.SuperStructure.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
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
    public SmartScore(
      SuperStructure structure,
      EndEffector endEffector,
      Objective objective,
      BooleanSupplier ejectGamePiece
      ){

        Supplier< SuperStructureState> superStructureSupplier = 
        ()-> getSuperStructureTarget(objective);
        Supplier<Boolean> targetEjectType = 
        ()-> getEjectType(objective);

 


        var SuperStructureCommand = structure.acceptSuperStructureState(superStructureSupplier);
        var ejectCommand = Commands.run(()-> endEffector.ejectOverridePiece(targetEjectType.get()), endEffector);

        addCommands(
          Commands.parallel(
            SuperStructureCommand,
            Commands.waitUntil(ejectGamePiece).andThen(
              ejectCommand
            )

          )
        );



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
  public static boolean getEjectType(Objective target){
    if (target.isConeNode()){
      return true;
    }else return false;
      
  }

  public static SuperStructureState getSuperStructureTarget(Objective target){
    switch (target.nodeLevel){ 

      case HYBRID:
        return SuperStructurePresets.groundIntake;
  
      case MID: {
       if(target.isConeNode()){
        return SuperStructurePresets.midCone;
       }else
       return SuperStructurePresets.midCube;
       
      }


      case HIGH: 
      if(target.isConeNode()){
        return SuperStructurePresets.highCone;
       }else
       return SuperStructurePresets.highCube;

       default: 
       return SuperStructurePresets.stowed;
      }
    
      
      
    }


        
         
    
    
  

    public static Pose2d getDriveTarget(
      Pose2d robotPose, Objective objective) {
    var pose = robotPose;
    var nodeTranslation = getNodeTranslation(objective).toTranslation2d();
    
    return new Pose2d();
    




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
