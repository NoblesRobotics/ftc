package org.firstinspires.ftc.teamcode.nobles.swervetest;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl.ACCEL_CONSTRAINT;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl.HEADING_PID;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl.TRANSLATIONAL_PID;
import static org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl.VEL_CONSTRAINT;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

import java.util.ArrayList;
import java.util.List;

public class SwerveDriveImpl extends SwerveDrive {
    private final IMU imu;
    private final SwerveModule[] modules;
    private final TrajectoryFollower follower;
    private final TrajectorySequenceRunner runner;
    private final List<Integer> lastEncPositions = new ArrayList<>(), lastEncVels = new ArrayList<>();

    public SwerveDriveImpl(HardwareMap hardwareMap) {
        super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, DriveConstants.TRACK_WIDTH, DriveConstants.TRACK_WIDTH);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                DriveConstants.MOTOR_VELO_PID.p, DriveConstants.MOTOR_VELO_PID.i, DriveConstants.MOTOR_VELO_PID.d,
                DriveConstants.MOTOR_VELO_PID.f * 12 / batteryVoltageSensor.getVoltage()
        );

        modules = new SwerveModule[] {
                new SwerveModule(hardwareMap, 2, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 3, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 0, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 1, DcMotorSimple.Direction.FORWARD)
        };

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        runner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );
    }

    protected double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void setMotorPowers(double v0, double v1, double v2, double v3) {
        modules[0].setPower(v0);
        modules[1].setPower(v1);
        modules[2].setPower(v2);
        modules[3].setPower(v3);
    }

    public void setModuleOrientations(double r0, double r1, double r2, double r3) {
        modules[0].setAngle(Math.toDegrees(r0));
        modules[1].setAngle(Math.toDegrees(r1));
        modules[2].setAngle(Math.toDegrees(r2));
        modules[3].setAngle(Math.toDegrees(r3));
    }

    @NonNull
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();
        List<Double> wheelPositions = new ArrayList<>();
        for (SwerveModule module : modules) {
            int position = module.getPosition();
            lastEncPositions.add(position);
            wheelPositions.add(encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @NonNull
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();
        List<Double> wheelVelocities = new ArrayList<>();
        for (SwerveModule module : modules) {
            int vel = (int) module.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @NonNull
    public List<Double> getModuleOrientations() {
        List<Double> list = new ArrayList<>();
        for (SwerveModule module : modules) list.add(Math.toRadians(module.getAngle()));
        return list;
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = runner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) {
            setDriveSignal(signal);
            for (SwerveModule module : modules) module.updateServo();
        }
    }

    public void followTrajectory(Trajectory trajectory) {
        runner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
        while (!Thread.currentThread().isInterrupted() && runner.isBusy()) update();
    }

    public void calibrateServos() {
        for (SwerveModule module : modules) module.calibrateServo();
    }
}

/*class SwerveModule {
    public SwerveModule() {}

    public void setPower(double power) {}

    public void setOrientation(double targetOrientation) {
        double error = subtractAngles(getOrientation(), targetOrientation), lastError = error, integralSum = 0;
        ElapsedTime timer = new ElapsedTime();

        while ((error = subtractAngles(getOrientation(), targetOrientation)) > 2) {
            double derivative = (error - lastError) / timer.seconds();
            integralSum += (error * timer.seconds());

            double pidOut = servoCoefficients.p * error + servoCoefficients.i * integralSum + servoCoefficients.d * derivative;
            servo.setPower(Math.signum(error) * pidOut);

            lastError = error;
            timer.reset();
        }
    }

    public double getPosition() {
        return DriveConstants.encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getOrientation() {
        return servoEncoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}*/