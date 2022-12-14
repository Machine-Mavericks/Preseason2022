// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeedCommand extends CommandBase {
  double FEEDSPEED = 0.85;
  Feeder feeder;
  long dTime;

  /** Creates a new FeedCommand. */
  public FeedCommand(Feeder feeder) {
    addRequirements(feeder);
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.dTime = System.currentTimeMillis()+4000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.setFeederSpeed(FEEDSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis()>=dTime;
  }
}
