// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023.SubSystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
    public static final Command spining = null;
    /** Creates a new newintake. */
    public intake() {}

    int motor1id = 0;
    int motor2id = 1;
    int motor3id = 3;

    boolean debounce;
    int kP = 0;
    int kI = 0;
    int kD = 0;
    int setpoint;
    boolean open = false;
    CANSparkMax motor1 = new CANSparkMax(motor1id, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(motor2id, MotorType.kBrushless);
    CANSparkMax motor3 = new CANSparkMax(motor3id, MotorType.kBrushless);
    RelativeEncoder m_encoder;
    PIDController pid = new PIDController(kP, kI, kD);

    public void start() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor2.follow(motor1);
        motor1.set(0.5);
    }

    public void change() {
        motor1.restoreFactoryDefaults();
        motor2.restoreFactoryDefaults();
        motor1.set(0.5);
        motor2.set(-0.5);
    }

    public void stop() {
        motor1.set(0);
        motor2.set(0);
    }

    public void close() {
        if (open == false) {
            m_encoder = motor3.getEncoder();
            motor3.set(pid.calculate(m_encoder.getPosition(), setpoint));
            System.out.println(open);
            open = true;
        } else {
            m_encoder = motor3.getEncoder();
            motor3.set(pid.calculate(m_encoder.getPosition(), setpoint));
            System.out.println(open);
            open = false;
        }
    }

    public CommandBase toggle() {

        return runOnce(() -> change());
    }

    public CommandBase movemotorCommand() {

        return runOnce(() -> start());
    }

    public CommandBase stopmotorCommand() {

        return runOnce(() -> stop());
    }

    public CommandBase closeInkate() {

        return runOnce(() -> close());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
}
