package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

// Boilerplate code for april tag detection with FTC vision portal
public class AprilTagFinder {
    private final VisionPortal portal;
    private final AprilTagProcessor processor;

    public AprilTagFinder(HardwareMap hardwareMap) {
        processor = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processor)
                .build();
    }

    public TransformedDetection getDetection(int tagId) {
        for ( AprilTagDetection detection : processor.getDetections() ) {
            if (detection.ftcPose != null && detection.id == tagId) return new TransformedDetection(detection);
        }
        return null;
    }

    static class TransformedDetection {
        public final double range, x;

        public TransformedDetection(AprilTagDetection detection) {
            // Apply a transformation if necessary (e.g. if the camera is not centered on the robot)
            range = detection.ftcPose.range;
            x = detection.ftcPose.x;
        }
    }
}
