package ca.warp7.frc2023.subsystems;
import static ca.warp7.frc2023.Constants.*;
import ca.warp7.frc2023.lib.SparkMaxUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fourbar extends SubsystemBase {
    private static Fourbar instance;

    public static Fourbar getInstance() {
        if (instance == null) instance = new Fourbar();
        return instance;
    }

    // creates a motorgroup to hold both motors, this is important as different commands could break the fourbar
    public CANSparkMax motorMain = SparkMaxUtil.createMasterMotor(FOURBAR_MOTOR_PORT_0);
    public CANSparkMax motorSecondary = SparkMaxUtil.createFollowerMotor(motorMain, FOURBAR_MOTOR_PORT_1, true); // need inversion, stops death

    // the encoder (incremental)
    public RelativeEncoder encoder = motorMain.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192); // last value = count per rotation
    
    // create and set a PID controller
    public SparkMaxPIDController controllermotorMain = SparkMaxUtil.createPIDController(
        motorMain, encoder, 0.1, 1e-4, 1, 0, 2000, 1500 );
        
    // gets the number of rotations from the encoder
    public double getPosition() {
        return encoder.getPosition();
    }

    // Sets the setPoint of the PID controller to whatever was passed as the input
    public void setPosition(double length) {
        controllermotorMain.setReference(length, CANSparkMax.ControlType.kPosition);
    }
}

// please forgive our sins sir daniel
