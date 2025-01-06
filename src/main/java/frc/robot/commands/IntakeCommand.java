// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends Command {
  private final Intake intake;
  private boolean direction;
  private boolean stop;

  public IntakeCommand(Intake intake, boolean direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
    this.stop = false;
  }

  /** Creates a new SetIntakeSpeed. */
  public IntakeCommand(Intake intake, boolean direction, boolean stop) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
    this.stop = stop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!stop) {
      if (!direction) {
        intake.intake(0.5, 0.5);
      } else {
        intake.intake(-0.5, -0.5);
      }
    } else {
      intake.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}