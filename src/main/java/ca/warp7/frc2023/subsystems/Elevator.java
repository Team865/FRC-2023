package ca.warp7.frc2023.subsystems;

import static ca.warp7.frc2023.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    // returns the instance of the subsystem
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    public static CANSparkMax createMasterSparkMAX(int deviceID) {
        CANSparkMax master = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        master.restoreFactoryDefaults();
        master.setIdleMode(CANSparkMax.IdleMode.kBrake);
        master.enableVoltageCompensation(12.0);
        return master;
    }

    public CANSparkMax createFollowerSparkMAX(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults();
        follower.follow(LeftMain);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // creates an inverted SparkMAx follower motor
    public CANSparkMax createFollowerSparkMAXInverted(int deviceID) {
        CANSparkMax follower = new CANSparkMax(deviceID, CANSparkMaxLowLevel.MotorType.kBrushed);
        follower.restoreFactoryDefaults()  ;
        follower.follow(LeftMain, true);
        follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        follower.enableVoltageCompensation(12.0);
        return follower;
    }

    // makes a leader and a follower motor for the left side
    public CANSparkMax LeftMain = createMasterSparkMAX(kElevatorLeftMain);
    public CANSparkMax LeftSecondary = createFollowerSparkMAX(kElevatorLeftSecondary);

    // makes the inverted follower motors for the right side
    public CANSparkMax RightMain = createFollowerSparkMAXInverted(kElevatorRightMain);
    public CANSparkMax RightSecondary = createFollowerSparkMAXInverted(kElevatorRightSecondary);
}
