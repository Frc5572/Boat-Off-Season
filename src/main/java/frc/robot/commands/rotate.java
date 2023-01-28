package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class rotate extends CommandBase {
    private CANSparkMax max;
    private DutyCycleEncoder dEncoder;
    private PIDController pid_controller = new PIDController(-(3.0 / 360.0), 0, 0.01 / 360.0);
    private double goal;
    private final double encOffset = 0.949;

    public rotate(CANSparkMax max, DutyCycleEncoder dEncoder, double angle) {
        this.max = max;
        this.dEncoder = dEncoder;
        this.goal = angle;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double v = pid_controller.calculate(encVal() * 360, goal);
        System.out.println("" + v + " " + (encVal() * 360) + " " + goal);
        max.set(v);

    }

    @Override
    public void end(boolean interrupt) {
        max.set(0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs((encVal() * 360) - goal) < 3;
    }

    public double encVal() {
        return -(dEncoder.getAbsolutePosition() - encOffset);
    }
}
