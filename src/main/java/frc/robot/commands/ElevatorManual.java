package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorManual extends Command {
    
    private final Elevator elevator;
    private final double speed;

    public ElevatorManual(Elevator elevator, double speed) {
        this.elevator = elevator;
        this.speed = speed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setSpeed(speed);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
