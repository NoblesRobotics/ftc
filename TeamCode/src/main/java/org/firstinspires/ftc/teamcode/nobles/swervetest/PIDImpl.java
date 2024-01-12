package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDImpl {
    private final double kp, ki, kd;

    public PIDImpl(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    private final ElapsedTime timer = new ElapsedTime();

    public void reset() {
        timer.reset();
    }

    private double integralSum = 0, lastError = 0;

    public double update(double error) {
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double out = kp * error + ki * integralSum + kd * derivative;
        lastError = error;
        timer.reset();
        return out;
    }
}
