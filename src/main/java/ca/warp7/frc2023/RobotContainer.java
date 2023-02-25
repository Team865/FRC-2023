// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

    // create a chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();

        // add options to the chooser
        m_chooser.setDefaultOption("Default Auto", Commands.print("Default Auto"));
        m_chooser.addOption("Other auto", null);

        // put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    private void configureBindings() {}

    public Command getAutonomousCommand() {
        // return the selected command
        return m_chooser.getSelected();
    }
}
