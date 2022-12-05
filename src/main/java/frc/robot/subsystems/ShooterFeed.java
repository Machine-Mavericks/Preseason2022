// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFeed extends SubsystemBase {
  TalonSRX feedMot;
  /** Creates a new ShooterFeed. */
  public ShooterFeed(TalonSRX feedMot) {
    this.feedMot =feedMot;
    
    
  }
  public void setSpeed(double speed){
    feedMot.set(ControlMode.PercentOutput, speed);

  }

  public void stopfeed(){
		feedMot.set(ControlMode.PercentOutput, 0);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
