// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class FeedCommand extends CommandBase {

  /** The shooter feed subsystem */
  private Feeder feeder;

  /** The speed to drive the feeder at */
  private double speed;

  /** 
   * Creates a new ShootCommand. 
   * @param feeder The shooter feed subsystem
   * @param speed The speed to drive the feeder at
  */
  public FeedCommand(Feeder feeder, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
    this.feeder = feeder;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    feeder.setfeederOutput(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
