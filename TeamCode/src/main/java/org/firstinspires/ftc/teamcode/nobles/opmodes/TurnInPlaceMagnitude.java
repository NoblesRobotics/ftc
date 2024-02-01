package org.firstinspires.ftc.teamcode.nobles.opmodes;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.slide.DoubleSlide;
import org.firstinspires.ftc.teamcode.nobles.swerve.SimpleIMU;
import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveModule;

@TeleOp
public class TurnInPlaceMagnitude extends LinearOpMode {
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

        while (opModeIsActive()) {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x,
                            Rotation2d.fromDegrees(imu.getAngle())
                    )
            );

            double maxSpeed = 0;
            for (SwerveModuleState state : states) {
                if (Math.abs(state.speedMetersPerSecond) > maxSpeed) maxSpeed = Math.abs(state.speedMetersPerSecond);
            }

            double magnitude = Math.max(
                    Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)),
                    Math.abs(gamepad1.right_stick_x)
            );

            for (int i = 0; i < 4; i++) {
                modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                modules[i].updateServo();
                modules[i].setPower(states[servoToStateMap[i]].speedMetersPerSecond / maxSpeed * magnitude); // DOES MAPPING APPLY???
            }
        }
    }
}
