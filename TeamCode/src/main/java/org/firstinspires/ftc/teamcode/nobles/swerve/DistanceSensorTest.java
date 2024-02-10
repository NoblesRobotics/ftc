package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
