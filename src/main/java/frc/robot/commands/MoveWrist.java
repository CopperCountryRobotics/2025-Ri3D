package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;

public class MoveWrist extends Command {
    
    private final Wrist wrist;
    private final boolean direction;

    public MoveWrist(Wrist wrist, boolean direction) {
        this.wrist = wrist;
        this.direction = direction;
    }

    @Override
    public void execute() {
        wrist.rotate(WristConstants.wristSpeed * (direction ? 1 : -1));
    }

    @Override
    public void end(boolean interrupted){ 
        wrist.rotate(0);
        wrist.setPosition(wrist.getPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
