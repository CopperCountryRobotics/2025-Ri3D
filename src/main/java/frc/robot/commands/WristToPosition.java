// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class WristToPosition extends Command {
  Wrist wrist;
  double pos;
  /** Creates a new WristToPosition. */
  public WristToPosition(Wrist wrist, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    addRequirements(wrist);
    this.pos = pos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      wrist.setPosition(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atPosition();
  }
}
