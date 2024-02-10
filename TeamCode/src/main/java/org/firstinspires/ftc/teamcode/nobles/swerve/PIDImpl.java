package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.util.ElapsedTime;

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
