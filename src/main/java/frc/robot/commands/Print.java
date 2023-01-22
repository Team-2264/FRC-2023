package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Print extends CommandBase {

    private String message;

    public Print(String message) {
        this.message = message;
    }

    @Override
    public void initialize() {
        System.out.println(message);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
