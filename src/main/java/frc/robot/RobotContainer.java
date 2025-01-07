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
import frc.robot.commands.RobotHome;
import frc.robot.commands.ScoreLevelThree;
import frc.robot.commands.ScoreLevelTwo;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.flicker.Flicker;
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
  private final Flicker flicker = new Flicker();
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

    //Flicker Wheel
    opController.axisGreaterThan(2, .5).onTrue(Commands.run(()->flicker.spinWheel(.5), flicker)).onFalse(Commands.run(()->flicker.spinWheel(0), flicker));
    opController.axisLessThan(2, -.5).onTrue(Commands.run(()->flicker.spinWheel(-.5), flicker)).onFalse(Commands.run(()->flicker.spinWheel(0), flicker));

    //Flicker Arm
    opController.povUp().onTrue(Commands.run(()->flicker.moveArm(.5), flicker)).onFalse(Commands.run(()->flicker.moveArm(0), flicker));
    opController.povDown().onTrue(Commands.run(()->flicker.moveArm(-.5), flicker)).onFalse(Commands.run(()->flicker.moveArm(0), flicker));

    //Wrist
    opController.povLeft().whileTrue(new MoveWrist(wrist, true));
    opController.povRight().whileTrue(new MoveWrist(wrist, false));

    //Arm
    opController.axisGreaterThan(0, .5).whileTrue(new ArmCommand(arm, .3));
    opController.axisLessThan(0, -.5).whileTrue(new ArmCommand(arm, -.3));

    //Intake
    opController.button(7).whileTrue(new IntakeCommand(intake, false));
    opController.button(8).whileTrue(new IntakeCommand(intake, true));

    //Elevator
    opController.button(1).onTrue(new ElevatorManual(elevator, .1)).onFalse(new ElevatorManual(elevator,0));
    opController.button(2).onTrue(new ElevatorManual(elevator, -.1)).onFalse(new ElevatorManual(elevator,0));

    //Encoder
    opController.button(3).onTrue(new ResetEncoders(elevator, wrist, arm, flicker));

    driverController.button(1).onTrue(new RobotHome(arm, elevator, wrist));
    driverController.button(2).onTrue(new ScoreLevelThree(arm, elevator, wrist));
    driverController.button(3).onTrue(new ScoreLevelTwo(arm, elevator, wrist));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

//TODO: Hold wrist in 0 position
//TODO: Get wrist encoder position for 90 degree turn
//TODO: Get arm encoder position for top scoring position
//TODO: Tune arm pid loop 
//TODO: Get elevator position for middle score 
//TODO: Tune elevator PID loop
