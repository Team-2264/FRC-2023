package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Delay extends CommandBase {

    private double delay;
    private long startTime;

    public Delay(double delayTime) {
        this.delay = delayTime;
    }

    @Override
    public void initialize() {
        System.out.println("Delaying for " + delay + " seconds");
        this.startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return (System.currentTimeMillis() - startTime > (delay * 1000));
    }
}
