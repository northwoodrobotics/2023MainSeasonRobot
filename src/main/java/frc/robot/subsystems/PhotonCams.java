package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.ExternalLib.SpectrumLib.drivers.PhotonColorSensors;

public class PhotonCams extends SubsystemBase{
    
    // creates photonVision Camera object 
    private final PhotonCamera visionCam;
    // Tag Id being tested
    private static int targetTag = 3;
    // the position of the camera relative to the the robot's center of rotatation.
    private static Transform3d robotToCam = new Transform3d(new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(5),Units.inchesToMeters(6.5)), new Rotation3d(0, 0, Units.degreesToRadians(270)));
    // position of the goal, relative to the target Tag
    private static Transform3d tagToGoal = new Transform3d(new Translation3d(-1, 0, Units.inchesToMeters(22)), new Rotation3d());
    // the location of the tag, relative to the location of the robot.
    private Pose2d tagLocation = new Pose2d();
    // Derived Path to Tag
    private PathPlannerTrajectory Route2Tag;
    // 3d Pose of robot, Constructed from 2d Odometry Pose
    private Pose3d robot3D;
    // internal Pose object for Path Generation
    private Pose2d TagPose;
    // Vector to Tag
    private Transform2d robotToTag;

    public PhotonCams(PhotonCamera camera){
        //sets the camera in the subsytem, as the camera given. 
        this.visionCam = camera;
        // turns off "driver mode"
        visionCam.setDriverMode(false);
        // turns off LEDs
        visionCam.setLED(VisionLEDMode.kOff);


        
    }
    // returns the position of the tag, relative to the robot
    public Pose2d getTagLocation(Pose2d robotPose){
         robot3D = new Pose3d(robotPose.getX(), robotPose.getY(), 0, new Rotation3d(0, 0, robotPose.getRotation().getRadians()))  ;
        //gets latest data from camera
        var res = visionCam.getLatestResult();
        // checks if the camera has a target
        if(res.hasTargets()){
            var targetOpt = res.getBestTarget().getFiducialId();
            // checks if the the tag is Tag 3
            if(targetOpt == targetTag){
                // grabs best camera data.
                var camToTarget = res.getBestTarget().getBestCameraToTarget();
              
                // translates robot position to camera position
                var cameraPose = robot3D.transformBy(robotToCam.inverse());
            
                //calculates target position
                Pose3d TargetPose = cameraPose.transformBy(camToTarget);
                
                // sets internal location to the calculated location which is 1m in front of the tag position
                tagLocation = TargetPose.transformBy(tagToGoal).toPose2d();
                
                // spits out target location, which is 1m in front of the tag position
                return TargetPose.transformBy(tagToGoal).toPose2d();
               

            }else // if the tag is not the correct one, then spit out the last known location
            return tagLocation;
            
        } 
        else // if no tag is detected, spit out last known tag location
    return tagLocation;
    }
    public PathPlannerTrajectory calculateTrajectorToTag(Pose2d RobotPose){
        TagPose = getTagLocation(RobotPose);

        // calculates a 2d vector in between the robot and Tag
        robotToTag = new Transform2d(RobotPose, TagPose);

        // feeds all data into path generation software
        Route2Tag = PathPlanner.generatePath(
            // these are acceleration and velocity constraints, in m/s and m/s squared
            new PathConstraints(1, 0.5), 
            // PathPoints have 3 values, the cordinates of the intial point, the heading of the desired vector, and the "holonomic rotation" of the robot
            new PathPoint(RobotPose.getTranslation(),robotToTag.getRotation(),RobotPose.getRotation() ), 
            new PathPoint(TagPose.getTranslation(), robotToTag.getRotation(), TagPose.getRotation())
            );
        return Route2Tag;
    }

    // spits out last known tag location
    public Pose2d getLastTagLocation(){
        return tagLocation;
        
    }
    // returns detected tag ID
    public int getTagID(){
        var res = visionCam.getLatestResult();
        if(res.hasTargets()){
          return  res.getBestTarget().getFiducialId();
        }else return 0;
    }
    // returns if the camera has a target
    public boolean hasTargets(){
        var res = visionCam.getLatestResult();
        return res.hasTargets();
    }
    



    @Override 
    public void periodic(){
        
    }

}