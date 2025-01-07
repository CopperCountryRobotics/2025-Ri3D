// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Target {
    public static final Target ZERO = new Target(0.0, 0.0, 0.0, 0.0);

    public static final Target topPosition = new Target(0.0, 0.0, 0.0, 0.0);
    public static final Target middlePosition = new Target(0.0, 0.0, 0.0, 0.0);
    public static final Target bottomPosition = new Target(0.0, 0.0, 8.0, 0.0);

    public final double arm;
    public final double elevator;
    public final double intake;
    public final double wrist;

    public Target(double arm, double elevator, double intake, double wrist) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.wrist = wrist;
    }

    @Override
    public String toString() {
        return "Target [arm =" + arm + ", elevator =" + elevator + ", intake =" + intake + ", wrist =" + wrist + "]";
    }
}
