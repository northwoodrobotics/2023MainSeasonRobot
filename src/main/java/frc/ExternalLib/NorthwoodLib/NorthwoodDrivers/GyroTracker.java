package frc.ExternalLib.NorthwoodLib.NorthwoodDrivers;


import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.util.WPIUtilJNI;
 
    /** 
     * class for non Kalman Filter IMU based localization. This object tracks a robots position relative to the field using
     * the IMU's accererometer data. This can be used to enhance Localization through Odometry or Pose Estimators, 
     * as it is only suceptible to noise, from the gyro itself, rather than multiple sources like Odometry. Keep in mind, 
     * this should not be used in PLACE of Odometry, only to supliment it. 
    */
public class GyroTracker {
    private Pose2d m_poseMeters; 
    private double m_prevTimeSeconds = -1; 
    private Translation2d m_velocityVector;
    private Translation2d m_previousVelocity;

    private Rotation2d m_gyroOffset;
    private Rotation2d m_previousAngle;

   /** Constructs a GyroTracker Object
    * 
    @param gyroAngle: angle reported by the Gyroscope 
    @param InitialPose Starting Position on the field 

    
    */ 

    public GyroTracker(Rotation2d gyroAngle, Pose2d initalPose){
        m_poseMeters = initalPose; 
        m_gyroOffset = initalPose.getRotation().minus(gyroAngle);
        m_previousAngle = initalPose.getRotation();
          

    }
    /**
   * Constructs a Gyrotracker object with the default pose at the origin.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   */
    public GyroTracker(Rotation2d gyroAngle){
        this(gyroAngle, new Pose2d());
    }

     /**
   * Resets the robot's position on the field.
   *
   * <p>The gyroscope angle does not need to be reset here on the user's robot code. The library
   * automatically takes care of offsetting the gyro angle.
   *
   * @param pose The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
    public void resetPosition(Pose2d pose, Rotation2d gyroAngle){
        m_poseMeters = pose;
        m_previousAngle = pose.getRotation();
        m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    }
    /** 
     * Returns the Robot's current Velocity through at the current time using physics
    */
    // the acceleration Array MUST be in M/s. Most Gyro's output data in Gs, or in Gs with specific notation, BE careful of that! 
    public Translation2d velocityWithTime(short[] accelerationArray,double currentTimeSeconds){
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        m_prevTimeSeconds = currentTimeSeconds;
        m_previousVelocity = m_velocityVector;

        m_velocityVector = new Translation2d(accelerationArray[0]* period +m_previousVelocity.getX(), accelerationArray[1]*period+m_previousVelocity.getY());
        
        return m_velocityVector;


    }
    /** 
     * Returns the Robot's current Velocity through at the current time using physics
    */
    // the acceleration Array MUST be in M/s. Most Gyro's output data in Gs, or in Gs with specific notation, BE careful of that! 
    public Translation2d updateVelocity(short[] accelerationArray){
        return velocityWithTime(accelerationArray, WPIUtilJNI.now()*1.0e-6);
    }
    
  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method takes in the current time as a parameter to calculate period (difference
   * between two timestamps). The period is used to calculate the change in distance from a
   * velocity. This also takes in an angle parameter which is used instead of the angular rate that
   * is calculated from forward kinematics.
   *
   * @param currentTimeSeconds The current time in seconds.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param accelerationArray current Acceleration of robot
   *    
   * @return The new pose of the robot.
   */

    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, short[] accelerationArray) {
        double period = m_prevTimeSeconds >= 0 ? currentTimeSeconds - m_prevTimeSeconds : 0.0;
        m_prevTimeSeconds = currentTimeSeconds;
        updateVelocity(accelerationArray);
    
        var angle = gyroAngle.plus(m_gyroOffset);
    
        
        var newPose =
            m_poseMeters.exp(
                new Twist2d(
                    (m_velocityVector.getX() * period) + (accelerationArray[0]*Math.pow(period, 2)),
                    (m_velocityVector.getY() * period) + (accelerationArray[1]*Math.pow(period, 2)),
                    angle.minus(m_previousAngle).getRadians()));
    
        m_previousAngle = angle;
        m_poseMeters = new Pose2d(newPose.getTranslation(), angle);
    
        return m_poseMeters;
    }
    
  /**
   * Updates the robot's position on the field using forward kinematics and integration of the pose
   * over time. This method automatically calculates the current time to calculate period
   * (difference between two timestamps). The period is used to calculate the change in distance
   * from a velocity. This also takes in an angle parameter which is used instead of the angular
   * rate that is calculated from forward kinematics.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param accelerationArray current Acceleration of robot
   * @return The new pose of the robot.
   */

    public Pose2d update(Rotation2d gyroAngle, short[] accelerationArray){
        return updateWithTime(WPIUtilJNI.now()* 1.0e-6, gyroAngle, accelerationArray);
    }




     
    /**
     * Returns the position of the robot on the field.
     *
     * @return The pose of the robot (x and y are in meters).
    */
    public Pose2d getPoseMeters() {
        return m_poseMeters;
    }







    


}
