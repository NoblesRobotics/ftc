package org.firstinspires.ftc.teamcode;

public class SinglePress {
    private final PressCallback callback;

    public SinglePress(PressCallback callback) {
        this.callback = callback;
    }

    private boolean lastState = false;

    public void update(boolean state) {
        // If this is run continuously, passing in the value of a button, the callback will only run
        // if the value goes from false to true, which happens which the button is pressed
        if (!lastState && state) callback.onPress();
        lastState = state;
    }

    public interface PressCallback {
        void onPress();
    }
}
