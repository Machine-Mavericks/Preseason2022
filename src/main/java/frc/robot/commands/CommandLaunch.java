// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeed;
public class CommandLaunch extends CommandBase {
  
  Shooter shooter;

  ShooterFeed feed;

  DoubleSupplier shootSpeedSupplier;

  DoubleSupplier feedSpeedDoubleSupplier;

  Long delay;

  /** Creates a new CommandLaunch. */
  public CommandLaunch(Shooter shooter, DoubleSupplier shootSpeedSupplier, ShooterFeed feed, DoubleSupplier feedSpeedSupplier) {
    addRequirements(shooter);

    addRequirements(feed);

    this.shooter = shooter;
    this.shootSpeedSupplier = shootSpeedSupplier;
    this.feed = feed;
    this.feedSpeedDoubleSupplier = feedSpeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopShoot();

    delay = System.currentTimeMillis() + 500; //five hundred means half a second
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get the motors moving for the shooter
    short break before 
    Get the feed motor moving */
    shooter.setSpeed(shootSpeedSupplier.getAsDouble());

    if (System.currentTimeMillis() > delay){
      
      feed.setSpeed(feedSpeedDoubleSupplier.getAsDouble());

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feed.stopfeed();

   // Delay(60);

    shooter.stopShoot();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
