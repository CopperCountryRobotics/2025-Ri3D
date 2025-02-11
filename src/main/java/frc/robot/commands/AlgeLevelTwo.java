// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.flicker.Flicker;
import frc.robot.subsystems.flicker.FlickerConstants;
import frc.robot.subsystems.wrist.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgeLevelTwo extends SequentialCommandGroup {
  /** Creates a new AlgeLevelThree. */
  public AlgeLevelTwo(Arm arm, Elevator elevator, Wrist wrist, Flicker flicker) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristToPosition(wrist, 0),
      new ArmToPosition(arm, ArmConstants.armVertical),// about straight up in the air
      new ElevatorToPosition(elevator, ElevatorConstants.bottomHeightAlge) 
      //new FlickerArmToPosition(flicker, FlickerConstants.pivotExtended)
    );
  }
}
