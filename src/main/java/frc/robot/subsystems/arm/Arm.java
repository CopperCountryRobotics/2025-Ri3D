// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {
    private CANSparkMax arm;
    private SparkPIDController pidControllerArm;
    private SparkAbsoluteEncoder encoderArm;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kPosition;

     public Arm(int armTwist) {
        this.arm = new CANSparkMax(armTwist, MotorType.kBrushless);

        this.arm.setSmartCurrentLimit(30); // recalculate (placeholder numbers)
        this.arm.setInverted(false);
        this.arm.setIdleMode(IdleMode.kBrake);


        encoderArm = arm.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoderArm.setPositionConversionFactor(ArmConstants.kArmEncoderRot2Rad);
        encoderArm.setVelocityConversionFactor(ArmConstants.kArmEncoderRPM2RadPerSec);
        //encoderArm.setPosition(Math.toRadians(-90.0));

        pidControllerArm = arm.getPIDController();

        kP = 0.55; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = .5; 
        kMinOutput = -.5;
    


        pidControllerArm.setP(0.55); // recalculate (placeholder numbers)
        pidControllerArm.setI(0.0);
        pidControllerArm.setD(0.0);
        ////pidControllerArm.setIZone(0);
        //pidControllerArm.setIMaxAccum(0.0, 0);
        pidControllerArm.setFF(0.0);
        pidControllerArm.setOutputRange(-.3, .3);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", encoderArm.getPosition());
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double encoderValue = SmartDashboard.getNumber("Set Rotations", 0);

        if((p != kP)) { pidControllerArm.setP(p); kP = p; }
        if((i != kI)) { pidControllerArm.setI(i); kI = i; }
        if((d != kD)) { pidControllerArm.setD(d); kD = d; }

        if((max != kMaxOutput) || (min != kMinOutput)) { 
            pidControllerArm.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }
        if((encoderValue != kPosition)){pidControllerArm.setReference(encoderValue, ControlType.kPosition); kPosition = encoderValue;}
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
        pidControllerArm.setReference(radians, ControlType.kPosition, 0); // recalculate (placeholder numbers)
    }
}