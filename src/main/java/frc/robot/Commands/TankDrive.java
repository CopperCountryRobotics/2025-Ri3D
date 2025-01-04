// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class TankDrive extends Command {

  private Drivetrain m_drive;
  private DoubleSupplier m_leftSpeed, m_rightSpeed;

  /** Creates a new TankDrive. */
  public TankDrive(Drivetrain drive, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    addRequirements(drive);
    m_drive = drive;
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.tankDrive(m_leftSpeed.getAsDouble(), m_rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
