package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import ca.warp7.frc2023.lib.SparkMaxUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Fourbar extends SubsystemBase {
    private static Fourbar instance;

    public static Fourbar getInstance() {
        if (instance == null) instance = new Fourbar();
        return instance;
    }

    // creates a motorgroup to hold both motors, this is important as different commands could break the fourbar
    private CANSparkMax motorMain = SparkMaxUtil.createMasterMotor(FOURBAR_MOTOR_PORT_0);
    private CANSparkMax motorSecondary = SparkMaxUtil.createFollowerMotor(motorMain, FOURBAR_MOTOR_PORT_1, true); // need inversion, stops death

    // the encoder (incremental)
    private Encoder encoder = (Encoder) motorMain.getEncoder();

    // create and set a PID controller
    private SparkMaxPIDController controllermotorMain =
            SparkMaxUtil.createPIDController(motorMain, encoder, 0.1, 1e-4, 0, 0, 2000, 1500);
    /* Current Values
     *    p = 0.1
     *    i = 1e-4
     *    d = 0
     *    kMinVel = 0
     *    kMaxVel = 2000
     *    kMaxAcc = 1500
     */

    // gets the number of rotations from the encoder
    public double getPosition() {
        return ((RelativeEncoder) encoder).getPosition();
    }

    // Sets the setPoint of the PID controller to whatever was passed as the input
    public void setPosition(double length) {
        controllermotorMain.setReference(length, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // update position to smartdashboard
        SmartDashboard.putNumber("Fourbar Position", getPosition());
    }
}

// please forgive our sins sir daniel
