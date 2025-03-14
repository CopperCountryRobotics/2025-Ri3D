// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.auton.ArmAuton;
import frc.robot.auton.DriveDistance;
import frc.robot.auton.WristAuton;
import frc.robot.commands.AlgeLevelThree;
import frc.robot.commands.AlgeLevelTwo;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.FlickerRemoveAlge;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakePulse;
import frc.robot.commands.MoveWrist;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.RobotHome;
import frc.robot.commands.ScoreLevelOne;
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

  private final SendableChooser<Command> chooser = new SendableChooser<Command>();


  private final Drivetrain drive = new Drivetrain();
  private final Elevator elevator = new Elevator();
  private final Flicker flicker = new Flicker();
  private final Intake intake = new Intake(9);
  private final Wrist wrist = new Wrist();
  private final Arm arm = new Arm(ArmConstants.armCANIDs);


  public RobotContainer() {

    driverController = new CommandJoystick(0);
    opController = new CommandJoystick(1);

    drive.setDefaultCommand(new ArcadeDriveCommand(drive, ()->driverController.getRawAxis(1), ()->driverController.getRawAxis(4), ()->driverController.button(6).getAsBoolean()));
    intake.setDefaultCommand(new IntakePulse(intake));
    configureBindings();

    // doSomething in auton (PURELY for examples, needs to be changed)
    chooser.setDefaultOption("Do Something", driveAndScore());
    chooser.addOption("Drive Dist", new DriveDistance(drive, .5, Units.inchesToMeters(75)));

    SmartDashboard.putData(chooser);
  }

  // Moving arm and wrist autonomously - can be used in both auton or be coded into a button on the controllers (PURELY for examples, needs to be changed)
  private Command driveAndScore() {
    return new ScoreLevelOne(arm, elevator, wrist).andThen(new DriveDistance(drive, .5, Units.inchesToMeters(75))).andThen(new ScoreLevelOne(arm, elevator, wrist));//.andThen(new IntakeCommand(intake, -.50).withTimeout(.25)).andThen(new WaitCommand(2)).andThen(new RobotHome(arm, elevator, wrist, flicker));
  }

  private void configureBindings() {

    // //Flicker Arm
    // opController.povUp().onTrue(Commands.run(()->flicker.moveArm(.5), flicker)).onFalse(Commands.run(()->flicker.moveArm(0), flicker));
    // opController.povDown().onTrue(Commands.run(()->flicker.moveArm(-.5), flicker)).onFalse(Commands.run(()->flicker.moveArm(0), flicker));

    // //Wrist
    opController.axisGreaterThan(2, .5).whileTrue(new MoveWrist(wrist, true));
    opController.axisLessThan(2, -.5).whileTrue(new MoveWrist(wrist, false));

    // //Arm
    opController.axisGreaterThan(3, .5).whileTrue(new ArmCommand(arm, .3));
    opController.axisLessThan(3, -.5).whileTrue(new ArmCommand(arm, -.3));

    // //Elevator
    opController.axisLessThan(1 , -.5).onTrue(new ElevatorManual(elevator, .1)).onFalse(new ElevatorManual(elevator,0));
    opController.axisGreaterThan(1 , .5).onTrue(new ElevatorManual(elevator, -.1)).onFalse(new ElevatorManual(elevator,0));

    //Encoder
    opController.button(9).onTrue(new ResetEncoders(elevator, wrist, arm, flicker)); //Change to start button

    //Scoring Positions
    opController.button(2).onTrue(new RobotHome(arm, elevator, wrist, flicker));
    opController.button(4).onTrue(new ScoreLevelThree(arm, elevator, wrist));
    opController.button(3).onTrue(new ScoreLevelTwo(arm, elevator, wrist));
    opController.button(1).onTrue(new ScoreLevelOne(arm, elevator, wrist));

    // //Intake
    driverController.axisGreaterThan(2, .6).whileTrue(new IntakeCommand(intake, .65));
    driverController.button(5).onTrue(Commands.either(new IntakeCommand(intake, -.35).withTimeout(.1), new IntakeCommand(intake, -1).withTimeout(.1), ()->Math.abs(wrist.getPosition()) > 1));

    //Alge positions
    opController.pov(180).onTrue(new AlgeLevelThree(arm, elevator, wrist, flicker));
    opController.pov(0).onTrue(new AlgeLevelTwo(arm, elevator, wrist, flicker));

    // //Flicker Wheel
    driverController.axisGreaterThan(3, .6).whileTrue(new FlickerRemoveAlge(flicker));

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
