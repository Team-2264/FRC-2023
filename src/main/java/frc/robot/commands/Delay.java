package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Delay extends CommandBase {

    private double delay;

    public Delay(double delayTime) {
        this.delay = delayTime;
    }

    @Override
    public void initialize() {
        System.out.println("Delaying for " + delay + " seconds");
    }

    @Override
    public void execute() {
        try {
            Thread.sleep((int) (delay * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
