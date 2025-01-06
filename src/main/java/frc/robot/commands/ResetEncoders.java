// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class ResetEncoders extends Command {
  Elevator elevator;
  Wrist wrist;
  /** Creates a new ResetEncoders. */
  public ResetEncoders(Elevator elevator, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.wrist = wrist;
    addRequirements(elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetEncoders();
    wrist.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
