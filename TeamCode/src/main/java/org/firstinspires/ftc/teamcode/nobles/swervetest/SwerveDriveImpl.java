package org.firstinspires.ftc.teamcode.nobles.swervetest;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.SwerveDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.ArrayList;
import java.util.List;

public class SwerveDriveImpl extends SwerveDrive {
    private final IMU imu;
    private final SwerveModule[] modules;

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
                new SwerveModule(hardwareMap, "frontLeft", true, compensatedCoefficients),
                new SwerveModule(hardwareMap, "rearLeft", true, compensatedCoefficients),
                new SwerveModule(hardwareMap, "rearRight", false, compensatedCoefficients),
                new SwerveModule(hardwareMap, "frontRight", false, compensatedCoefficients)
        };
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

    public void setModuleOrientations(double o0, double o1, double o2, double o3) {
        modules[0].setOrientation(o0);
        modules[1].setOrientation(o1);
        modules[2].setOrientation(o2);
        modules[3].setOrientation(o3);
    }

    @NonNull
    public List<Double> getWheelPositions() {
        List<Double> list = new ArrayList<>();
        for (SwerveModule module : modules) list.add(module.getPosition());
        return list;
    }

    @NonNull
    public List<Double> getModuleOrientations() {
        List<Double> list = new ArrayList<>();
        for (SwerveModule module : modules) list.add(module.getOrientation());
        return list;
    }
}

class SwerveModule {
    private final PIDCoefficients servoCoefficients = new PIDCoefficients(8, 0, 0);

    private final DcMotorEx motor;
    private final CRServo servo;
    private final AnalogInput servoEncoder;

    public SwerveModule(HardwareMap hardwareMap, String name, boolean reversed, PIDFCoefficients compensatedCoefficients) {
        motor = hardwareMap.get(DcMotorEx.class, name);

        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, compensatedCoefficients);
        if ( reversed ) motor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(CRServo.class, name + "Servo");
        servoEncoder = hardwareMap.get(AnalogInput.class, name + "Encoder");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

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

    private double subtractAngles(double from, double to) {
        if (from > to) return -subtractAngles(to, from);

        double normal = to - from;
        double around = from + (2 * Math.PI - to);
        if (normal <= around) return normal;
        else return -around;
    }
}