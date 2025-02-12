// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flicker.Flicker;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RobotHome extends SequentialCommandGroup {
  /** Creates a new ArmHome. */
  public RobotHome(Arm arm, Elevator elevator, Wrist wrist, Flicker flicker) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristToPosition(wrist, WristConstants.wristHome),
      new ElevatorToPosition(elevator, 0),
      new ArmToPosition(arm, 0), 
      new FlickerArmToPosition(flicker, 0)
    );
  }
}
