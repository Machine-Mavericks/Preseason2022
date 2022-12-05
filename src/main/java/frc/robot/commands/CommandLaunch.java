// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class CommandLaunch extends CommandBase {
  
  Shooter shooter;

  DoubleSupplier shootSpeedSupplier;

  /** Creates a new CommandLaunch. */
  public CommandLaunch(Shooter shooter, DoubleSupplier shootSpeedSupplier) {
    addRequirements(shooter);

    this.shooter = shooter;
    this.shootSpeedSupplier = shootSpeedSupplier;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopShoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setSpeed(shootSpeedSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShoot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
