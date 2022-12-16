// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class LowShootCommand extends CommandBase {
  double IDLESPEED = 0.4;
  double SHOOTERSPEED = 0.56;
  Shooter shooter;
  long dTime;

  /** Creates a new ShootCommand. */
  public LowShootCommand(Shooter shooter) {
    addRequirements(shooter);
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dTime = System.currentTimeMillis()+4000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterSpeed(SHOOTERSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.idleShooter(IDLESPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis()>=dTime;
  }
}
