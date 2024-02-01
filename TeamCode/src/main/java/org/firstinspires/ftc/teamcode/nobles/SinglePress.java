package org.firstinspires.ftc.teamcode.nobles;

public class SinglePress {
    private final PressCallback callback;

    public SinglePress(PressCallback callback) {
        this.callback = callback;
    }

    private boolean lastState = false;

    public void update(boolean state) {
        if (!lastState && state) callback.onPress();

        lastState = state;
    }

    public interface PressCallback {
        void onPress();
    }
}
