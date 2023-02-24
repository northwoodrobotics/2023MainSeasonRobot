package frc.robot.subsystems.SuperStructure;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructureSim extends SuperStructureBase{
    private Mechanism2d base;
    private MechanismRoot2d root;
    private MechanismLigament2d elevator;
    private MechanismRoot2d carraidge;
    private MechanismLigament2d wrist;
    SuperStructureSim(){
        root = base.getRoot("ElevatorBase", Units.inchesToMeters(26.134), Units.inchesToMeters(9.116));
        elevator = root.append(
            new MechanismLigament2d("Elevator", 
            Units.inchesToMeters(62.3),
            45.0 
                )
            );
        wrist = elevator.append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12.0),90.0));

    }

    
    public boolean hasGamePiece(){
        return hasGamePiece;
    }
    public boolean getIntakeStateChange(){
        return intakeStateHasChanged;
    }
    



  



    



  

            }