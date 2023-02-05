// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2023.SubSystems;

import java.util.concurrent.TimeUnit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase {
  public static final Command spining = null;
  /** Creates a new newintake. */
  public intake() {}
 
  int toggle = 0;

  
  public void change()  {
    if (toggle == 0) {
      System.out.println(toggle);
      toggle = 1;
      return;
    }
    if (toggle == 1) {
      System.out.println(toggle);
      toggle = 0;
    }

  }
  public CommandBase toggle() {

    return runOnce(() -> change());
    
  
  }
  public CommandBase movemotorCommand() {
    if (toggle == 0) {
      System.out.println("togglw");
      return runOnce(() -> System.out.println("run"));
    }

    return runOnce(() -> System.out.println("bob"));
    
  
  }
  public CommandBase stopmotorCommand() {
  
    return runOnce(() -> System.out.println("stoping"));
    
  }
  public CommandBase reversemotorCommand() {
  
    return runOnce(() -> System.out.println("reverseing"));
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
