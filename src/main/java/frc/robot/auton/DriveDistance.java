// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import edu.wpi.first.util.ErrorMessages;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivetrain;

public class DriveDistance extends Command {
  Drivetrain drive;
  double distance;
  double start; 
  double target;
  double speed;
  double error;
  double currentPos;
  double command;

  double Kp = 10;
  /**
   * 
   * @param drive Drivetrain subsystem
   * @param speed speed at which to drive (always positive)
   * @param dist distance to drive (negative is backwards)
   */
  public DriveDistance(Drivetrain drive, double speed, double dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.speed = speed;

    distance = dist;

    
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = (drive.getEncoders()[0] + drive.getEncoders()[1])/2;
    target = start + distance; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos = (drive.getEncoders()[0] + drive.getEncoders()[1])/2;
    error = target - currentPos;

    command = error * Kp; 
    
    if(Math.abs(command) < .25){
      command = Math.signum(command) * .25;
    }

    if(Math.abs(command) > speed){
      command = Math.signum(command) * speed;
    }

    drive.tankDrive(command, command);
    SmartDashboard.putNumber("Drive error", error);
    SmartDashboard.putNumber("Drive Encoder", currentPos);
    SmartDashboard.putNumber("Drive Command", command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < .0254;
  }
}
