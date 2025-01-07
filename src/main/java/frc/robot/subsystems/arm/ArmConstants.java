// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

/** Add your docs here. */
public class ArmConstants {
    public static final int armCANIDs = 10;

    public static final double kArmMotorGearRatio = 1.0 / 40.0; // recalculate (placholder numbers)
    public static final double kArmEncoderRot2Rad = kArmMotorGearRatio * 2.0 *
            Math.PI; // recalculate (placholder numbers)
    public static final double kArmEncoderRPM2RadPerSec = kArmEncoderRot2Rad
            / 60.0; // recalculate (placholder numbers)
    public static final double kPArm = 0.0;
    public static final double kIArm = 0.0;
    public static final double kDArm = 0.0;

    public static final boolean kArmEncoderReversed = true;    

    public static final double armLevelThree = 8.55;
    public static final double armLevelTwo = 8.55;
}