package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

import java.util.ArrayList;
import java.util.List;

public class SwerveServo {
    public final CRServo servo;
    public final AnalogInput analog;
    public double refPosition;

    public int currentRevs = 0;
    public double currentPosition = 0, targetPosition = 0;
    public SwerveServo(HardwareMap hardwareMap, int index) {
        servo = hardwareMap.get(CRServo.class, "servo" + index);
        analog = hardwareMap.get(AnalogInput.class, "analog" + index);
    }

    public void calibrate() {
        servo.setPower(0);
        refPosition = currentPosition = targetPosition = getSubrev();
    }

    public void update() {
        updatePosition();

        if (Math.abs(targetPosition - currentPosition) > 0.05) servo.setPower(-Math.signum(targetPosition - currentPosition) * 0.5);
        else servo.setPower(0);
    }

    enum ServoState {
        LOW,
        MID,
        HIGH
    }

    private double lastGoodSubrev = 0;
    public int badTimeout = 0;
    public ServoState state = ServoState.MID;

    private void updatePosition() {
        double subrev = getSubrev();

        if (Math.abs(deltaSubrev(lastGoodSubrev, subrev)) > 0.25) {
            if (badTimeout < 15) {
                subrev = lastGoodSubrev;
                badTimeout++;
            } else {
                lastGoodSubrev = subrev;
            }
        } else {
            lastGoodSubrev = subrev;
            badTimeout = 0;
        }

        switch (state) {
            case LOW:
                if (subrev > 0.333 && subrev < 0.666) state = ServoState.MID;
                else if (subrev > 0.666) {
                    currentRevs--;
                    state = ServoState.HIGH;
                }
                break;
            case MID:
                if (subrev < 0.333) state = ServoState.LOW;
                else if (subrev > 0.666) state = ServoState.HIGH;
                break;
            case HIGH:
                if (subrev < 0.333) {
                    currentRevs++;
                    state = ServoState.LOW;
                }
                else if (subrev > 0.333 && subrev < 0.666) state = ServoState.MID;
                break;
        }

        currentPosition = currentRevs + subrev;
    }

    private double getSubrev() {
        return analog.getVoltage() / 3.3;
    }

    private double deltaSubrev(double a, double b) {
        if (a > b) return -deltaSubrev(b, a);

        double normal = -(b - a);
        double around = (1 - b) + a;
        if (Math.abs(normal) <= Math.abs(around)) return normal;
        else return around;
    }

    public double getAngle() {
        return (currentPosition - refPosition) * 180.;
    }

    public void setAngle(double degrees) {
        double position = degrees / 180.;
        targetPosition = (position * 1.25) + refPosition;
    }

    public boolean isMoving() {
        return Math.abs(targetPosition - currentPosition) > 0.05;
    }
}
