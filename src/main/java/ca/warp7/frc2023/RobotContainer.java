// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import ca.warp7.frc2023.Commands.spining;
import ca.warp7.frc2023.SubSystems.intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    intake intakeing = new intake();
    spining spin = new spining(intakeing);
    CommandXboxController exampleCommandController = new CommandXboxController(1); 
    Trigger xButton = exampleCommandController.x(); 

    public RobotContainer() {
        configureBindings();
    }

    
    private void configureBindings() {
     exampleCommandController.x().toggleOnTrue(intakeing.movemotorCommand());
     exampleCommandController.a().toggleOnTrue(intakeing.stopmotorCommand());
     exampleCommandController.b().toggleOnTrue(intakeing.reversemotorCommand());
       
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
