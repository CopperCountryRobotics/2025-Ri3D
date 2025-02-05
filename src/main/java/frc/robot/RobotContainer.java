// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;

public class RobotContainer {
  /**
   * axis =
   * 0: Get the X axis value of left stick of the controller.
   * Left[-1,1]Right
   *
   * 1: Get the Y axis value of left stick of the controller.
   * Up[-1,1]Down
   *
   * 2: Get the axis value of left trigger of the controller.
   * Natural[0,1]Depressed
   *
   * 3: Get the axis value of right trigger of the controller.
   * Natural[0,1]Depressed
   *
   * 4: Get the X axis value of right stick of the controller.
   * Left[-1,1]Right
   *
   * 5: Get the Y axis value of right stick of the controller.
   * Up[-1,1]Down
   */
  private CommandJoystick driverController;
  private CommandJoystick opController;
  

  private final Drivetrain drive = new Drivetrain();

  private final Elevator elevate = new Elevator();


  public RobotContainer() {

    driverController = new CommandJoystick(0);
    opController = new CommandJoystick(1);

    drive.setDefaultCommand(new TankDriveCommand(drive, ()->driverController.getRawAxis(1), ()->driverController.getRawAxis(5)));
    new JoystickButton(opController.getHID(), 6).onTrue((new ElevatorCommand(elevate, true).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));
    new JoystickButton(opController.getHID(), 5).onTrue((new ElevatorCommand(elevate, false).withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
