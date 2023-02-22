package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;


public class PhotonCams extends SubsystemBase{
    
    // creates photonVision Camera object 
    private final PhotonCamera visionCam;

    // create Pose Estimator Object
    private final PhotonPoseEstimator poseEstimator;
    
    // create Field Layout Drawing
    private AprilTagFieldLayout layout;
 
   

    public PhotonCams(){
        //sets the camera in the subsytem, as the camera given. 
        this.visionCam = new PhotonCamera("Main");;
        // turns off "driver mode"
        visionCam.setDriverMode(false);
        // turns off LEDs
        visionCam.setLED(VisionLEDMode.kOff);
      /*  try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            layout.setOrigin(alliance == Alliance.Blue ?
                OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
          } catch(IOException e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            layout = null;
          } */

          final AprilTag tag01 = new AprilTag(1,
          new Pose3d(new Pose2d(new Translation2d(Units.inchesToMeters(118), Units.inchesToMeters(7.75)), Rotation2d.fromDegrees(180))));
          final AprilTag tag02 = new AprilTag(2,
          new Pose3d(new Pose2d(new Translation2d(Units.inchesToMeters(118), Units.inchesToMeters(40.25)), Rotation2d.fromDegrees(180))));
        ArrayList<AprilTag> list = new ArrayList<>();
        list.add(tag01); 
        list.add(tag02);  
        layout = new AprilTagFieldLayout(list, Units.inchesToMeters(118), Units.inchesToMeters(86));

        this.poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, visionCam, VisionConstants.robotToCam );

        
    }
    
    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        
        poseEstimator.setReferencePose(prevEstimatedRobotPose);
        return poseEstimator.update();
    }
   
    // returns if the camera has a target
    public boolean hasTargets(){
        var res = visionCam.getLatestResult();
        return res.hasTargets();
    }
    



    @Override 
    public void periodic(){
        Optional<EstimatedRobotPose> result =
        getEstimatedGlobalPose(RobotContainer.m_SwerveSubsystem.dt.getPose());
        if (result.isPresent()){
            Logger.getInstance().recordOutput("VisionPose", result.get().estimatedPose.toPose2d());
        }
     
    }

}