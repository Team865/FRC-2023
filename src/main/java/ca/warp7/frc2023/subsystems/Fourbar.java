package ca.warp7.frc2023.subsystems;
import static ca.warp7.frc2023.Constants.*;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Fourbar extends SubsystemBase {
    /* fourbar subsytem, used as base to move with PID and remain in requested location
    TO DO:
        - setup PID
        - get PID to automove
        - account for gravity */
    private final TalonFX[] motors = { new TalonFX(FOURBAR_MOTOR_PORT_0), new TalonFX(FOURBAR_MOTOR_PORT_1) };
    private static Fourbar instance;
    
    public static Fourbar getInstance() {
        if (instance == null) instance = new Fourbar();
        return instance;
    }

    private void setMotors(double percent) {
        // motors are connected, both should spin when needed
        motors[0].set(ControlMode.PercentOutput, percent);
        motors[1].set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void periodic() {
    }

    @Override  
    public void simulationPeriodic() {}
}
