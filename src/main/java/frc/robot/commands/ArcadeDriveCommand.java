// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class ArcadeDriveCommand extends Command {

  private Drivetrain drive;
  private DoubleSupplier leftSupplier, rightSupplier;
  private BooleanSupplier slow;

  /** Creates a new TankDrive. */
  public ArcadeDriveCommand(Drivetrain drive, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed, BooleanSupplier slow) {
    addRequirements(drive);
    this.drive = drive;
    this.leftSupplier = leftSpeed;
    this.rightSupplier = rightSpeed;
    this.slow = slow;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble(), slow.getAsBoolean());
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
