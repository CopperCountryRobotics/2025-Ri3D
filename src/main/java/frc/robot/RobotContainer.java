// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.wrist.Wrist;

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
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake(9);
  private final Wrist wrist = new Wrist();
  private final Arm arm = new Arm(ArmConstants.armCANIDs);


  public RobotContainer() {

    driverController = new CommandJoystick(0);
    opController = new CommandJoystick(1);

    drive.setDefaultCommand(new TankDriveCommand(drive, ()->driverController.getRawAxis(1), ()->driverController.getRawAxis(3)));
    configureBindings();
  }

  private void configureBindings() {
    //opController.povUp().onTrue(new ElevatorToPosition(elevator, ElevatorConstants.topHeight));
    //opController.povDown().onTrue(new ElevatorToPosition(elevator, ElevatorConstants.bottomHeight));

    opController.povLeft().whileTrue(new MoveWrist(wrist, true));
    opController.povRight().whileTrue(new MoveWrist(wrist, false));

    opController.axisGreaterThan(0, .5).whileTrue(new ArmCommand(arm, .3));
    opController.axisLessThan(0, -.5).whileTrue(new ArmCommand(arm, -.3));

    opController.button(7).onTrue(new IntakeCommand(intake, false));
    opController.button(8).onTrue(new IntakeCommand(intake, true));

    opController.button(1).onTrue(new ElevatorManual(elevator, ()->.3)).onFalse(new ElevatorManual(elevator, ()->0));
    opController.button(2).onTrue(new ElevatorManual(elevator, ()->-.3)).onFalse(new ElevatorManual(elevator, ()->0));

    opController.button(3).onTrue(new ResetEncoders(elevator, wrist));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
