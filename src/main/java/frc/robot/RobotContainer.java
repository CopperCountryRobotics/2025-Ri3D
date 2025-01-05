// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {

  private CommandJoystick driverController;

  private final Drivetrain drive = new Drivetrain();


  public RobotContainer() {

    driverController = new CommandJoystick(0);

    drive.setDefaultCommand(new TankDriveCommand(drive, ()->driverController.getRawAxis(1), ()->driverController.getRawAxis(5)));

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
