package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.util.ElapsedTime;

// Implementation of a PID controller for driving in auto driving
// https://www.ctrlaltftc.com/the-pid-controller
public class PIDImpl {
    private final Coefs coefs;

    private ElapsedTime timer = new ElapsedTime();
    private double previousError = 1;

    public PIDImpl(Coefs coefs) {
        this.coefs = coefs;
    }

    public double update(double error) {
        double p = coefs.kp * error,
                i = coefs.ki * error * timer.seconds(),
                d = coefs.kd * (error - previousError) / timer.seconds(),
                output = p + i + d;

        timer.reset();
        previousError = error;

        return output;
    }

    public static class Coefs {
        public final double kp, ki, kd;

        public Coefs(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }
    }
}
