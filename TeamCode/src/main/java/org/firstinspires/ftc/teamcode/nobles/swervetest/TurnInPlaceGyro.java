package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.slidetest.DoubleSlide;

@TeleOp
public class TurnInPlaceGyro extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;

        SwerveModule[] modules = new SwerveModule[] {
                new SwerveModule(hardwareMap, 0, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 1, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 2, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 3, DcMotorSimple.Direction.REVERSE)
        };
        int[] servoToStateMap = new int[] {3, 1, 0, 2};

        Servo droneLauncher = hardwareMap.get(Servo.class, "servo4");
        DoubleSlide slide = new DoubleSlide(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "motor6");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "servo5");
        Servo rightFlap = hardwareMap.get(Servo.class, "servo6");
        Servo leftFlap = hardwareMap.get(Servo.class, "servo7");
        Servo pivot = hardwareMap.get(Servo.class, "servo8");
        SimpleIMU imu = new SimpleIMU(hardwareMap);

        ElapsedTime calibrationTimer = new ElapsedTime();
        while (!isStarted() || calibrationTimer.seconds() < 3) {
            for (SwerveModule module : modules) module.calibrateServo();
        }
        waitForStart();

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.203, 0.203),
                new Translation2d(0.203, -0.203),
                new Translation2d(-0.203, 0.203),
                new Translation2d(-0.203, -0.203)
        );
        double forwardAngle = 0;
        DriveStateA driveState = DriveStateA.NOT_MOVING;

        while (opModeIsActive()) {
            double currentAngle = imu.getAngle();
            //if (gamepad1.right_stick_x > 0) forwardAngle = currentAngle - 5;
            //else if (gamepad1.right_stick_x < 0) forwardAngle = currentAngle + 5;
            //forwardAngle += -gamepad1.right_stick_x * 2;

            if (gamepad1.y) forwardAngle = currentAngle;

            double turnFactor = -deltaAngle(forwardAngle, currentAngle) * 0.01;
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x != 0 ? gamepad1.right_stick_x : (Math.abs(turnFactor) > 0.1 ? Range.clip(turnFactor, -0.5, 0.5) : 0),
                            Rotation2d.fromDegrees(currentAngle)
                    )
            );

            double maxSpeed = 0;
            for (SwerveModuleState state : states) {
                if (Math.abs(state.speedMetersPerSecond) > maxSpeed) maxSpeed = Math.abs(state.speedMetersPerSecond);
            }

            boolean isAnyInput = gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0;
            double magnitude = isAnyInput ? 1 : Math.abs(turnFactor);

            switch (driveState) {
                case NOT_MOVING:
                    for (SwerveModule module : modules) {
                        module.updateServo();
                        module.setPower(0);
                    }

                    if (isAnyInput) driveState = DriveStateA.TURNING;
                    break;

                case TURNING:
                    boolean anyServoMoving = false;
                    for (int i = 0; i < 4; i++) {
                        modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                        modules[i].updateServo();
                        modules[i].setPower(0);
                        anyServoMoving |= modules[i].isServoMoving();
                    }

                    if (!anyServoMoving) driveState = DriveStateA.DRIVING;
                    break;

                case DRIVING:
                    for (int i = 0; i < 4; i++) {
                        modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                        modules[i].updateServo();
                        modules[i].setPower(states[servoToStateMap[i]].speedMetersPerSecond / maxSpeed * magnitude); // DOES MAPPING APPLY???
                    }

                    if (!isAnyInput) driveState = DriveStateA.NOT_MOVING;
                    break;
            }
        }
    }

    enum DriveStateA {
        NOT_MOVING,
        TURNING,
        DRIVING
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
}
