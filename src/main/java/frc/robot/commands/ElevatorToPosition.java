package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorToPosition extends Command {
    
    private final Elevator elevator;
    private final double height;
    private DoubleSupplier manual;

    public ElevatorToPosition(Elevator elevator, double height, DoubleSupplier manual) {
        this.elevator = elevator;
        this.height = height;
        this.manual = manual;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPosition(height);
    }

    @Override
    public boolean isFinished() {
        //if we hit a hard stop, cancel the command
        return elevator.hasHitHardStop() || Math.abs(manual.getAsDouble()) > .2;
    }

}
