// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmAuton extends Command {
  public final PIDController thetaController = new PIDController(0.1, 0.0, 0.0);
  private final Arm armAuton;
  private double armSpeedAuton;

  public ArmAuton(Arm armAuton, double armSpeedAuton) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armAuton = armAuton;
    this.armSpeedAuton = armSpeedAuton;
    addRequirements(armAuton);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
    armAuton.armTwist(armSpeedAuton);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armAuton.armTwist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
