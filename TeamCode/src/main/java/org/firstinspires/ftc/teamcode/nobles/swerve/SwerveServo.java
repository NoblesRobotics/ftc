package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwerveServo {
    private final double TOLERANCE = 0.02;
    private final double REV_RATIO = 221 / 180.;

    public final CRServo servo;
    public final AnalogInput analog;
    public double refPosition;

    public int currentRevs = 0;
    public double currentPosition = 0, targetPosition = 0;
    private double lastGoodSubrev = 0;
    private final int index;
    public SwerveServo(HardwareMap hardwareMap, int index) {
        servo = hardwareMap.get(CRServo.class, "servo" + index);
        analog = hardwareMap.get(AnalogInput.class, "analog" + index);
        this.index = index;
    }

    public void calibrate() {
        servo.setPower(0);
        targetPosition = lastGoodSubrev = getSubrev();
        if (SwerveServoStorage.hasCachedPositions) {
            refPosition = SwerveServoStorage.refPositions[index];
            currentRevs = SwerveServoStorage.currentRevs[index];
        } else {
            refPosition = SwerveServoStorage.refPositions[index] = lastGoodSubrev;
            currentRevs = 0;
        }
        currentPosition = currentRevs + lastGoodSubrev;
        /*if (currentPosition < 0.333) state = ServoState.LOW;
        else if (currentPosition < 0.666) state = ServoState.MID;
        else state = ServoState.HIGH;*/
    }

    public void update() {
        updatePosition();
        if (Math.abs(targetPosition - currentPosition) > TOLERANCE) {
            double velocity = Math.max(Math.min(Math.abs(targetPosition - currentPosition), 1), 0.125);
            servo.setPower(-Math.signum(targetPosition - currentPosition) * velocity);
        }
        else servo.setPower(0);
        SwerveServoStorage.currentRevs[index] = currentRevs;
    }

    enum ServoState {
        LOW,
        MID,
        HIGH
    }

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
        return (currentPosition - refPosition) / REV_RATIO * 180.;
    }

    public void setAngle(double degrees) {
        double position = degrees / 180.;
        targetPosition = (position * REV_RATIO) + refPosition;
    }

    public boolean isMoving() {
        return Math.abs(targetPosition - currentPosition) > TOLERANCE;
    }

    public boolean isErrorMajor() {
        return Math.abs(targetPosition - currentPosition) / REV_RATIO * 180. > 5;
    }
}
