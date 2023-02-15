// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023;

import ca.warp7.frc2023.Commands.spining;
import ca.warp7.frc2023.SubSystems.intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    intake intakeing = new intake();
    spining spin = new spining(intakeing);
    int toggle = 0;
    CommandXboxController controller = new CommandXboxController(1); 
   
 
    
    

    public RobotContainer() {

        configureBindings();
    }

    
    private void configureBindings() {
        controller.b().toggleOnTrue(intakeing.toggle());
        controller.x().toggleOnTrue(intakeing.movemotorCommand());
        controller.a().toggleOnTrue(intakeing.stopmotorCommand());
        
    

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
