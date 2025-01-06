// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private SparkPIDController pidControllerArm;
    private RelativeEncoder encoderArm;

     public Arm(int armTwist) {
        this.arm = new CANSparkMax(armTwist, MotorType.kBrushless);
        this.arm.setSmartCurrentLimit(30); // recalculate (placeholder numbers)
        this.arm.setInverted(false);
        this.arm.setIdleMode(IdleMode.kBrake);
        encoderArm = arm.getEncoder();
        encoderArm.setPositionConversionFactor(ArmConstants.kArmEncoderRot2Rad);
        encoderArm.setVelocityConversionFactor(ArmConstants.kArmEncoderRPM2RadPerSec);
        encoderArm.setPosition(Math.toRadians(-90.0));
        pidControllerArm = arm.getPIDController();
        pidControllerArm.setP(0.55); // recalculate (placeholder numbers)
        pidControllerArm.setI(0.0);
        pidControllerArm.setD(0.0);
        pidControllerArm.setIZone(0);
        pidControllerArm.setIMaxAccum(0.0, 0);
        pidControllerArm.setFF(0.0);
        pidControllerArm.setOutputRange(-.5, .5);
    }

    @Override
    public void periodic() {

    }

    public double getArmVelocity() {
        return encoderArm.getVelocity();
    }

    public Rotation2d getArmPosition() {
        return Rotation2d.fromRadians(encoderArm.getPosition());
    }

    public void armTwist(double armSpeed) {
        this.arm.set(armSpeed);
    }

    public void positionArm(double radians) {
        pidControllerArm.setReference(radians, ControlType.kPosition, 0, 0.028 * getArmPosition().getCos(),
                ArbFFUnits.kPercentOut); // recalculate (placeholder numbers)
    }
}