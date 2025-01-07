// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmToPosition extends Command {
  private Arm arm; 
  double pos; 
  /** Creates a new ArmToPosition. */
  public ArmToPosition(Arm arm, double pos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm; 
    this.pos = pos; 
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.positionArm(pos);
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
    return arm.atSetpoint();
  }
}