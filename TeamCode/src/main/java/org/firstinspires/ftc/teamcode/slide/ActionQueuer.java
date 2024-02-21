package org.firstinspires.ftc.teamcode.slide;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.Deque;

// Class that accepts a series of actions and runs them "asynchronously" as logn as update is
// continuously called
public class ActionQueuer {
    private final Deque<Action> queue = new ArrayDeque<>();
    private Action currentAction = null;

    public void add(Action... actions) {
        queue.addAll(Arrays.asList(actions));
    }

    public void update() {
        if (currentAction != null && currentAction.isComplete()) currentAction = null;

        if (currentAction == null && queue.size() > 0) {
            Action action = queue.pop();
            action.start();
            currentAction = action;
        }
    }

    public void clear() {
        queue.clear();
    }

    public void idleOnBusy() {
        // Only returns after all actions in the queue have completed
        update();
        while (currentAction != null) {
            update();
            Thread.yield();
        }
    }

    public static class SlideAction implements Action {
        private final DoubleSlide slide;
        private final int position;

        public SlideAction(DoubleSlide slide, int position) {
            this.slide = slide;
            this.position = position;
        }

        public void start() {
            slide.setPosition(position);
        }

        public boolean isComplete() {
            return !slide.isBusy();
        }
    }

    public static class ServoAction implements Action {
        private final Servo servo;
        private final double position;
        private final int delayMs;

        public ServoAction(Servo servo, double position, int delayMs) {
            this.servo = servo;
            this.position = position;
            this.delayMs = delayMs;
        }

        private final ElapsedTime timer = new ElapsedTime();

        public void start() {
            servo.setPosition(position);
            timer.reset();
        }

        public boolean isComplete() {
            return timer.milliseconds() >= delayMs;
        }
    }

    public interface Action {
        void start();
        boolean isComplete();
    }
}
