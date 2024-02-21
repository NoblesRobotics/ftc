package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoSwerveDrive extends SwerveDrive {
    private final double TICKS_PER_REV = 320, INCHES_PER_REV = Math.PI * 2.8;

    private double forwardAngle = 90.;
    private double posX, posY;
    private final int ySign;

    private final DistanceSensor distanceSensor;

    public AutoSwerveDrive(HardwareMap hardwareMap, double posX, double posY, int ySign) {
        super(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        this.posX = posX;
        this.posY = posY;
        if (ySign != 1 && ySign != -1) throw new RuntimeException("bad y sign");
        this.ySign = ySign;
    }

    private final double DECEL_MAX_ERROR = 0.2;
    private final int MOTION_MIN_TICK = 8, NO_MOTION_LOOP_LIMIT = 10;

    private final PIDImpl.Coefs DRIVE_PID_COEFS = new PIDImpl.Coefs(2, 0, 0.2);

    public void drive(ErrorFunction errorFunction) {
        // Create a PID controller for each wheel
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

        // Stop any drive if it takes too long
        while (timer.seconds() < 4) {
            int[] currentPositions = readAllPositions();

            // Create an array representing how far each wheel has run since the start
            // (and calculate the average distance traveled)
            int[] deltaTicks = new int[4];
            int avgDeltaTick = 0;
            for (int i = 0; i < 4; i++) {
                int deltaTick = Math.abs(currentPositions[i] - startPositions[i]);;
                deltaTicks[i] = deltaTick;
                avgDeltaTick += deltaTick / 4;
            }

            // Apply the provided error function to the distances the wheels have traveled
            // An error function like in driveToDistance might not use this distance information, it
            // would simply use data from the distance sensor
            double[] driveErrors = errorFunction.driveErrors(deltaTicks);
            if (driveErrors == null || driveErrors.length != 4) throw new RuntimeException("bad error function");
            double avgDriveError = 0;
            for (double driveError : driveErrors) avgDriveError += driveError / 4;

            // Apply the PIDs to the values from the error function
            for (int i = 0; i < 4; i++) {
                modules[i].setPower(pids[i].update(driveErrors[i]));
                modules[i].updateServo();
            }

            // If in the deceleration stage (last 20%) of the drive
            if (Math.abs(avgDriveError) < DECEL_MAX_ERROR) {
                // Then if we move in any given loop by less than 8 encoder ticks on average for 10
                // loops consecutively, we are done
                // (Motion by 8 ticks or less is practically invisible)
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
        // Convert inches to travel to ticks to travel
        int targetDeltaTick = (int) (Math.abs(inches) / INCHES_PER_REV * TICKS_PER_REV);

        drive(deltaTicks -> {
            // Each wheel needs to travel the correct amount of ticks
            // Calcualte what fraction is left to travel and multiply by the correct sign
            double[] errors = new double[4];
            for (int i = 0; i < 4; i++) errors[i] = (Math.signum(inches) * (targetDeltaTick - deltaTicks[i]) / targetDeltaTick);
            return errors;
        });
    }

    public void driveAtAngle(double dx, double dy) {
        // Trigonometry to drive along any straight line path
        setAngle(-Math.toDegrees(Math.atan2(dx, dy * ySign)) + forwardAngle);
        drive(-Math.hypot(dx, dy * ySign));
    }

    public void driveToPosition(double x, double y) {
        // Use a reference to the last position of the robot to drive on an absolute basis
        driveAtAngle(x - posX, y - posY);
        posX = x;
        posY = y;
    }

    public void snapPosition(double x, double y) {
        // Driving using any function other than driveToPosition will not update posX and posY, so
        // it may be necessary to recalibrate with the known position of the robot
        posX = x;
        posY = y;
    }

    public void driveToDistance(double targetInches) {
        // Use a "basis" of 12 inches as we can comfortably stop from full speed at that distance
        // If we actually start further than 12 inches, we'll run at full speed until 12 inches
        // If we start closer, we'll never run at full speed
        double initialOffset = 12 - targetInches;

        drive(deltaTicks -> {
            // Calculate error as how much initialOffset is left to traverse
            double distance = distanceSensor.getDistance(DistanceUnit.INCH);
            double offset = distance - targetInches;
            double error = Math.min(offset / initialOffset, 1);

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
        // Makes a diamond shape from the wheels, so that equal power to all wheels will spin robot
        double[] swerveAngles = {-45, 45, 135, -135};
        for (int i = 0; i < 4; i++) modules[i].setAngle(swerveAngles[i]);
        positionServos();

        PIDImpl pid = new PIDImpl(TURN_PID_COEFS);
        int loopsNoPower = 0;
        while (true) {
            double headingError = deltaAngle(imu.getAngle(), targetAngle) / 90;

            // Two ways to calculate power:
            // Use a PID (more correct)
            // Apply a constant power if more than 5 degrees away from correct value (more stable
            // but pretty slow)
            double headingOffset = deltaAngle(imu.getAngle(), targetAngle);
            double headingPower = Math.abs(headingOffset) > 5 ? Math.signum(headingOffset) * 0.35 : 0;
            //double headingPower = pid.update(headingError);

            for (SwerveModule module : modules) {
                module.setPower(headingPower);
                module.updateServo();
            }

            // Same as in drive: if decelerating and not applying enough power to actually move for
            // enough loops, stop the function
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
