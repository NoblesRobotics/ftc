package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

// Static functions allow access to telemetry object without having to pass it through parameters
public class AttributeTelemetry {
    private AttributeTelemetry() {}

    private static Telemetry telemetry;

    private static final Map<String, String> data = new HashMap<>();

    public static void setTelemetry(Telemetry telemetryP) {
        telemetry = telemetryP;
        telemetry.clear();
        data.clear();
    }

    public static void set(String key, String value) {
        data.put(key, value);
        update();
    }

    public static void remove(String key) {
        data.remove(key);
        update();
    }

    private static void update() {
        if (telemetry == null) return;

        for (Map.Entry<String, String> entry : data.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.update();
    }
}
