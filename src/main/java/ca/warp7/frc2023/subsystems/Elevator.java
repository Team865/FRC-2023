package ca.warp7.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    // returns the instance of the subsystem
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

}
