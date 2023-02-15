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

  CANSparkMax motor1 = new CANSparkMax(toggle, MotorType.kBrushless);
  CANSparkMax motor2 = new CANSparkMax(toggle, MotorType.kBrushless);
  public void start() {
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor2.follow(motor1);
    motor1.set(0.5);
  }
  public void change()  {
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor1.set(0.5);
    motor2.set(-0.5);


  }
  public void stop(){
    motor1.set(0);
    motor2.set(0);
  }
  public CommandBase toggle() {

    return runOnce(() -> change());
    
  
  }
  public CommandBase movemotorCommand() {
  
    return runOnce(() -> start());
  
  }
  public CommandBase stopmotorCommand() {
  
    return runOnce(() ->stop());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
