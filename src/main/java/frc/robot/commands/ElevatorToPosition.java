package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorToPosition extends Command {
    
    private final Elevator elevator;
    private final double height;

    public ElevatorToPosition(Elevator elevator, double height) {
        this.elevator = elevator;
        this.height = height;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setPosition(height);
    }

    @Override
    public boolean isFinished() {
        //if we hit a hard stop, cancel the command
        return elevator.hasHitHardStop();
    }

}
