package frc.robot.subsystems.SuperStructure;

import edu.wpi.first.math.util.Units;

public class SuperStructureState{
    public double elevatorPositionRadians;
    public double wristAngleRadians; 

    public SuperStructureState(double position, double wristPosition){
        this.elevatorPositionRadians = position;
        this.wristAngleRadians = wristPosition;
      
    }
    public SuperStructureState(){
        this.elevatorPositionRadians = 0;
        this.wristAngleRadians = 0;
      
    }
    
    public double getHeightDemand(){
        return this.elevatorPositionRadians;
    }
    
    public double getWristAngleRadians(){
        return this.wristAngleRadians;
    }
    public double getWristAngleDegrees(){
        return Units.radiansToDegrees(this.wristAngleRadians);
    }
    public void setHeightDemand(double radians){
        this.elevatorPositionRadians = radians;
    }
    public void setWristAngleRadians(double radians){
        this.wristAngleRadians = radians;
    }
    public void setWristAngleDegrees(double degrees){
        this.wristAngleRadians = Units.degreesToRadians(degrees);
    }
}
