// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  
  private TalonSRX shooterMotor;
  
  /** Creates a new Shooter. */
  public Shooter(TalonSRX shooterMotor) {
      this.shooterMotor = shooterMotor;
  }

  public void setOutput(double output){
    shooterMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
