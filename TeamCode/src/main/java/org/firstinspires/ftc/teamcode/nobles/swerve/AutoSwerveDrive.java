package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Config
public class AutoSwerveDrive extends SwerveDrive {
    private final double TICKS_PER_REV = 300, INCHES_PER_REV = Math.PI * 2.8;

    private double forwardAngle = 0., servoAngle = 0.;

    private final DistanceSensor distanceSensor;
    private final double voltageFactor;

    public AutoSwerveDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        voltageFactor = Range.clip(hardwareMap.voltageSensor.iterator().next().getVoltage() / 13.96, 1, 2);
    }

    private final double DECEL_MAX_ERROR = 0.2;
    private final int MOTION_MIN_TICK = 8, NO_MOTION_LOOP_LIMIT = 10;

    private final double DRIVE_P = 10, DRIVE_HEADING_P = 0.1, DRIVE_MAX_TIME_S = 4;

    public void drive(ErrorFunction errorFunction) {
        ElapsedTime timer = new ElapsedTime();
        int[] startPositions = readAllPositions();
        int[] imuAdjustSigns = getIMUAdjustSigns();

        int lastAvgDeltaTick = 0;
        int loopsNoMotion = 0;

        while (timer.seconds() < DRIVE_MAX_TIME_S) {
            double headingError = deltaAngle(imu.getAngle(), forwardAngle);
            double imuAdjustMagnitude = headingError * DRIVE_HEADING_P;

            int[] currentPositions = readAllPositions();

            int[] deltaTicks = new int[4];
            int avgDeltaTick = 0;
            for (int i = 0; i < 4; i++) {
                int deltaTick = Math.abs(currentPositions[i] - startPositions[i]);;
                deltaTicks[i] = deltaTick;
                avgDeltaTick += deltaTick / 4;
            }

            double[] driveErrors = errorFunction.driveErrors(deltaTicks);
            if (driveErrors == null || driveErrors.length != 4) throw new RuntimeException("bad error function");
            double avgDriveError = 0;
            for (double driveError : driveErrors) avgDriveError += driveError / 4;

            for (int i = 0; i < 4; i++) {
                double drivePower = Range.clip(driveErrors[i], -0.5, 0.5);
                modules[i].setPower(drivePower /*+ imuAdjustMagnitude * imuAdjustSigns[i]*/);
                modules[i].updateServo();
            }

            if (Math.abs(avgDriveError) < DECEL_MAX_ERROR) {
                if (Math.abs(avgDeltaTick - lastAvgDeltaTick) < MOTION_MIN_TICK) {
                    loopsNoMotion++;
                    if (loopsNoMotion > NO_MOTION_LOOP_LIMIT) break;
                } else {
                    loopsNoMotion = 0;
                }
            }
            lastAvgDeltaTick = avgDeltaTick;
        }

        for (SwerveModule module : modules) module.setPower(0);
        setAngle(servoAngle);
    }

    public void drive(double inches) {
        double inchesFactored = inches;
        int targetDeltaTick = (int) (Math.abs(inchesFactored) / INCHES_PER_REV * TICKS_PER_REV);
        drive(deltaTicks -> {
            double[] errors = new double[4];
            for (int i = 0; i < 4; i++) errors[i] = (Math.signum(inchesFactored) * (targetDeltaTick - deltaTicks[i]) / targetDeltaTick) * DRIVE_P * 2;
            return errors;
        });
    }

    public void driveToDistance(double targetInches, double initialInches) {
        double initialOffset = initialInches - targetInches;
        drive(deltaTicks -> {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            double offset = distance - targetInches;
            double error = (offset / initialOffset);
            System.out.println(distance + " " + offset + " " + error);

            double[] errors = new double[4];
            for (int i = 0; i < 4; i++) errors[i] = error;
            return errors;
        });
    }

    public interface ErrorFunction {
        double[] driveErrors(int[] deltaTicks);
    }

    public void turn(double targetAngle) {
        double[] swerveAngles = {-45, 45, 135, -135};
        for (int i = 0; i < 4; i++) modules[i].setAngle(swerveAngles[i]);
        positionServos();

        int loopsNoPower = 0;
        while (true) {
            double headingOffset = deltaAngle(imu.getAngle(), targetAngle);
            double headingPower = Math.abs(headingOffset) > 3 ? 0.3 * Math.signum(headingOffset) * voltageFactor : 0;
            System.out.println(headingOffset + " " + headingPower + " " + loopsNoPower);

            for (SwerveModule module : modules) {
                module.setPower(headingPower);
                module.updateServo();
            }

            if (headingPower == 0) {
                loopsNoPower++;
                if (loopsNoPower > NO_MOTION_LOOP_LIMIT) break;
            } else {
                loopsNoPower = 0;
            }
        }

        for (SwerveModule module : modules) module.setPower(0);
        setAngle(servoAngle);
        forwardAngle = targetAngle;
    }

    private int[] readAllPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
    }

    private int getAvgPosition() {
        int avg = 0;
        for (int i = 0; i < 4; i++) avg += modules[i].getPosition() / 4;
        return avg;
    }

    private double deltaAngle(double aDeg, double bDeg) {
        if (aDeg > bDeg) return -deltaAngle(bDeg, aDeg);

        aDeg += 180.;
        bDeg += 180.;
        double normal = -(bDeg - aDeg);
        double around = (360 - bDeg) + aDeg;

        if (Math.abs(normal) <= Math.abs(around)) return normal;
        else return around;
    }

    public void setAngle(double degrees) {
        if (!Arrays.asList(-90., 0., 90., 180.).contains(degrees)) throw new RuntimeException("invalid servo angle");

        for (SwerveModule module : modules) module.setAngle(degrees);
        positionServos();
        servoAngle = degrees;
    }

    private int[] getIMUAdjustSigns() {
        switch ((int) servoAngle) {
            case 0:
                return new int[] {1, 1, -1, -1};

            case -90:
                return new int[] {-1, 1, 1, -1};

            case 90:
                return new int[] {1, -1, -1, 1};

            case 180:
                return new int[] {-1, -1, 1, 1};
        }

        throw new RuntimeException("invalid servo angle");
    }
}
