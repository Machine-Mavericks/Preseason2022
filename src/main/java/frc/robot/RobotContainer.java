// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.instant.ExampleButtonCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /** Instance of the robot's drivetrain */
  private Drivetrain drivetrain = new Drivetrain(RobotMap.leftDriveControllers, RobotMap.rightDriveControllers);

  /** Instance of the robot's shooter */
  private Shooter shooter = new Shooter(RobotMap.topTalon1);
  /** Intance of the robot's feeder */
  private Feeder feeder = new Feeder(RobotMap.topTalon2);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure default commands (there's some debate about where this should be done)
    // For drivetrain, use a DriveCommand with the suppliers from OI
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain, OI.leftDriveSupplier, OI.rightDriveSupplier));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Make the ExampleButtonCommand run each time the example button is pressed
    OI.exampleButton.whenPressed(new ExampleButtonCommand());
    
    OI.shootButton.whileHeld(new ShootCommand(shooter, Shooter.SHOOT_SPEED));
    OI.feedButton.whileHeld(new FeedCommand(feeder, Feeder.FEED_SPEED));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
