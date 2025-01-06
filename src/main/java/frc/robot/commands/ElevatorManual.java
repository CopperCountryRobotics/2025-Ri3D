package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorManual extends Command {
    
    private final Elevator elevator;
    private final DoubleSupplier speed;

    public ElevatorManual(Elevator elevator, DoubleSupplier speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSpeed(speed.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
