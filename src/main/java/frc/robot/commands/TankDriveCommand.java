// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class TankDriveCommand extends Command {

  private Drivetrain drive;
  private DoubleSupplier leftSupplier, rightSupplier;

  /** Creates a new TankDrive. */
  public TankDriveCommand(Drivetrain drive, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    addRequirements(drive);
    this.drive = drive;
    this.leftSupplier = leftSpeed;
    this.rightSupplier = rightSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankDrive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
