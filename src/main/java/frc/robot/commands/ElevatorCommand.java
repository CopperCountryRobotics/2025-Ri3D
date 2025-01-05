// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommand extends Command {

  private Elevator elevate;
  private boolean goingUp;
  private final ArmPose[] poses = {new ArmPose(0, 0), new ArmPose(.25, 0), new ArmPose(.5, -.25)};
  private int currentPose;

  /** Creates a new TankDrive. */
  public ElevatorCommand(Elevator elevate, boolean goingUp) {
    addRequirements(elevate);
    this.elevate = elevate;
    this.goingUp = goingUp;
    currentPose = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (goingUp){
        if (currentPose != poses.length - 1){
            currentPose++;
        }
    }
    else{
        if (currentPose != 0){
            currentPose--;
        }
    }
    elevate.setPose(poses[currentPose].height, poses[currentPose].wristAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  /**
   * @param height
   * @param wristAngle
   */
  record ArmPose(double height, double wristAngle){}
}
