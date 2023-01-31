package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushless);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    public static CANSparkMax LeftMain = createMasterSparkMAX(kElevatorLeftMain);
    public static CANSparkMax LeftSecondary = createMasterSparkMAX(kElevatorLeftSecondary);

    public static CANSparkMax RightMain = createMasterSparkMAX(kElevatorLeftMain);
    public static CANSparkMax RightSecondary = createMasterSparkMAX(kElevatorLeftSecondary);
}
