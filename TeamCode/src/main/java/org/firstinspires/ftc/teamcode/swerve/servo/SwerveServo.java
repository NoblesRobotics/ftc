package org.firstinspires.ftc.teamcode.swerve.servo;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class SwerveServo {
    private final double TOLERANCE = 0.02;
    private final double REV_RATIO = 221 / 180.;

    private final CRServo servo;
    private final AnalogInput analog;
    private final int index;

    private double refPosition;
    private int currentRevs = 0;
    private double currentPosition = 0, targetPosition = 0;

    public SwerveServo(HardwareMap hardwareMap, int index) {
        servo = hardwareMap.get(CRServo.class, "servo" + index);
        analog = hardwareMap.get(AnalogInput.class, "analog" + index);
        this.index = index;
    }

    public void calibrate() {
        // The main purpose of calibrating the servo is to set refPosition
        // If there are no cached servo positions, then we assume the robot has been pre-aligned,
        // so we record the starting position as refPosition, and then we use refPosition as the
        // "zero point" in the rest of the code
        // If there are cached servo positions, we restore them
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
    }

    public void update() {
        // This has to be run continuously after a call to setAngle
        updatePosition();

        if (Math.abs(targetPosition - currentPosition) > TOLERANCE) {
            // Guarantee a minimum power value when rotation is necessary
            double velocity = Range.clip(Math.abs(targetPosition - currentPosition), 0.125, 1);
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
    public ServoState state = ServoState.MID;

    private double lastGoodSubrev = 0;
    private int badTimeout = 0;

    private void updatePosition() {
        double subrev = getSubrev();

        // When the absolute encoder wraps around, because it is voltage based the computer will
        // read in between values that are not valid
        // For example in between going from 0.99 to 0.01 subrevs we could read values like 0.7 and
        // 0.3
        // We need to filter out those values
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

        // This state machine is used to accurately detect servo wraparounds
        // If we were at 0.6 and then read 0.8, the servo simply traveled +0.2
        // If we were at 0.1 and then read 0.2, the servo traveled -0.3 and wrapped around, so we
        // need to account for one revolution in the negative direction
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
        // The analog encoder on the servo will send a voltage between 0 and 3.3 volts based on the
        // servo position
        return analog.getVoltage() / 3.3;
    }

    private double deltaSubrev(double a, double b) {
        // This accounts for the fact that (for example) the positions 0.9 and 0.1 should be
        // considered to be 0.2 apart, not 0.8 apart
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
        // If this condition is not met, we are sending 0 power to the servo in update()
        return Math.abs(targetPosition - currentPosition) > TOLERANCE;
    }
}
