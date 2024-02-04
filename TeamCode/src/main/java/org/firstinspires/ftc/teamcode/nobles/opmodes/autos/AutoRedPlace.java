package org.firstinspires.ftc.teamcode.nobles.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.opmodes.SwerveAuto;

@Autonomous
public class AutoRedPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);
        SwerveAuto auto = new SwerveAuto(hardwareMap, false, false, false);
        waitForStart();
        auto.start();
    }
}
