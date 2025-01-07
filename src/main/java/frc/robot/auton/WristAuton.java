// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;

public class WristAuton extends Command {
  public final PIDController thetaController = new PIDController(0.1, 0.0, 0.0);
  private final Wrist wristAuton;
  private double wristSpeedAuton;

  public WristAuton(Wrist wristAuton, double wristSpeedAuton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristAuton = wristAuton;
    this.wristSpeedAuton = wristSpeedAuton;
    addRequirements(wristAuton);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristAuton.rotate(wristSpeedAuton);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristAuton.rotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
