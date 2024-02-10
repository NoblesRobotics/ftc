package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class AutoSwerveDrive extends SwerveDrive {
    private final double TICKS_PER_REV = 320, INCHES_PER_REV = Math.PI * 2.8;

    private double forwardAngle = 90.;
    private double posX = 60, posY = 8.75;
    private final int xSign;

    private final DistanceSensor distanceSensor;

    public AutoSwerveDrive(HardwareMap hardwareMap, double posX, double posY, int xSign) {
        super(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        this.posX = posX;
        this.posY = posY;
        if (xSign != 1 && xSign != -1) throw new RuntimeException("bad x sign");
        this.xSign = xSign;
    }

    private final double DECEL_MAX_ERROR = 0.2;
    private final int MOTION_MIN_TICK = 8, NO_MOTION_LOOP_LIMIT = 10;

    private final PIDImpl.Coefs DRIVE_PID_COEFS = new PIDImpl.Coefs(2, 0, 0.2);

    public void drive(ErrorFunction errorFunction) {
        PIDImpl[] pids = new PIDImpl[] {
                new PIDImpl(DRIVE_PID_COEFS),
                new PIDImpl(DRIVE_PID_COEFS),
                new PIDImpl(DRIVE_PID_COEFS),
                new PIDImpl(DRIVE_PID_COEFS)
        };
        ElapsedTime timer = new ElapsedTime();
        int[] startPositions = readAllPositions();

        int lastAvgDeltaTick = 0;
        int loopsNoMotion = 0;

        while (timer.seconds() < 4) {
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
                modules[i].setPower(pids[i].update(driveErrors[i]));
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
    }

    public void drive(double inches) {
        int targetDeltaTick = (int) (Math.abs(inches) / INCHES_PER_REV * TICKS_PER_REV);
        drive(deltaTicks -> {
            double[] errors = new double[4];
            for (int i = 0; i < 4; i++) errors[i] = (Math.signum(inches) * (targetDeltaTick - deltaTicks[i]) / targetDeltaTick);
            return errors;
        });
    }

    public void driveAtAngle(double dx, double dy) {
        setAngle(-Math.toDegrees(Math.atan2(dx, dy * xSign)) + forwardAngle);
        drive(-Math.hypot(dx, dy * xSign));
    }

    public void driveToPosition(double x, double y) {
        driveAtAngle(x - posX, y - posY);
        posX = x;
        posY = y;
    }

    public void snapPosition(double x, double y) {
        posX = x;
        posY = y;
    }

    public void driveToDistance(double targetInches) {
        double initialOffset = 12 - targetInches;
        if (initialOffset > 100) return;
        drive(deltaTicks -> {
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            double offset = distance - targetInches;
            double error = Math.min(offset / initialOffset, 1);
            System.out.println(distance + " " + offset + " " + error);

            double[] errors = new double[4];
            for (int i = 0; i < 4; i++) errors[i] = -error;
            return errors;
        });
    }

    public interface ErrorFunction {
        double[] driveErrors(int[] deltaTicks);
    }

    private final PIDImpl.Coefs TURN_PID_COEFS = new PIDImpl.Coefs(1, 0, 0);

    private final double NO_MOTION_TURN_POWER = 0.2;

    public void turn(double targetAngle) {
        double[] swerveAngles = {-45, 45, 135, -135};
        for (int i = 0; i < 4; i++) modules[i].setAngle(swerveAngles[i]);
        positionServos();
        System.out.println("turn " + targetAngle);

        PIDImpl pid = new PIDImpl(TURN_PID_COEFS);
        int loopsNoPower = 0;
        while (true) {
            double headingError = deltaAngle(imu.getAngle(), targetAngle) / 90;
            //double headingPower = pid.update(headingError);
            double headingOffset = deltaAngle(imu.getAngle(), targetAngle);
            double headingPower = Math.abs(headingOffset) > 5 ? Math.signum(headingOffset) * 0.4 : 0;
            System.out.println(headingOffset + " " + headingPower);

            for (SwerveModule module : modules) {
                module.setPower(headingPower);
                module.updateServo();
            }

            if (Math.abs(headingError) < DECEL_MAX_ERROR) {
                if (Math.abs(headingPower) < NO_MOTION_TURN_POWER) {
                    loopsNoPower++;
                    if (loopsNoPower > NO_MOTION_LOOP_LIMIT) break;
                } else {
                    loopsNoPower = 0;
                }
            }
        }

        for (SwerveModule module : modules) module.setPower(0);
        //forwardAngle = targetAngle;
    }

    private int[] readAllPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getPosition();
        return positions;
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
        for (SwerveModule module : modules) module.setAngle(degrees);
        positionServos();
    }
}
