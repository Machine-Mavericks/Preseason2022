// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Drivetrain;


public class BalanceCommand extends CommandBase {
  private final double kP = -0.001;
  private final double kI = 0.01;
  private final double kD = 0.01;
  private final float setpoint = 0;
  PIDController pid = new PIDController(kP, kI, kD);

  // Use addRequirements() here to declare subsystem dependencies.
  Gyro gyro;
  Drivetrain drivetrain;
  
  /** Supplier for left side output percent */
  DoubleSupplier leftSupplier;
  /** Supplier for right side output percent */
  DoubleSupplier rightSupplier;
  /** Creates a new BalanceCommand. */
  public BalanceCommand(Gyro gyro, Drivetrain drivetrain) {
    this.gyro = gyro;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setOutput(pid.calculate(gyro.getPitch() - gyro.getZeroValue(), setpoint), pid.calculate(gyro.getPitch() - gyro.getZeroValue(), setpoint));
    System.out.println(gyro.getPitch() - gyro.getZeroValue());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
