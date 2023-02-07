package ca.warp7.frc2023.subsystems;
import static ca.warp7.frc2023.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Fourbar extends SubsystemBase {
    /* fourbar subsytem, used as base to move with PID and remain in requested location
    TODO:
        - setup PID
        - get PID to automove
        - account for gravity */
    private final TalonFX[] motors = { new TalonFX(FOURBAR_MOTOR_PORT_0), new TalonFX(FOURBAR_MOTOR_PORT_1), new TalonFX(FOURBAR_MOTOR_PORT_2), new TalonFX(FOURBAR_MOTOR_PORT_3) };
    private final PIDController pid = new PIDController(0, 0, 0);
    
    public Fourbar() {}

    @Override
    public void periodic() {}

    @Override  
    public void simulationPeriodic() {}
}
