package frc.util.dashboardUtil;

import java.time.LocalTime;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * A utility class for creating timers on the SchafferBoard.
 */
public class TimerWidget implements Sendable {
    /** An enumerator for the timer modes. */
    public enum Mode
    {
        // If more modes are needed, contact Elad. DO NOT ADD THEM ON YOUR OWN.

        /** Auto mode. Will display the timer as RED. */
        AUTO(1),
        /** Teleop mode. Will display the timer as YELLOW. */
        TELEOP(0);

        public final int value;

        Mode(int index) {
            value = index;
        }

        @Override
        public String toString() {
            return this.name();
        }
    }
    /**
     * Auto Mode. The timer will display as RED.
     */
    public static final int MODE_AUTO = 1;
    /**
     * Teleop Mode. The timer will display as YELLOW.
     */
    public static final int MODE_TELEOP = 0;
    private String name;
    private Mode mode;
    private double duration = 0;
    private long startTime = -99;

    /**
     * Creates a new TimerWidget.
     * @param id The id of the timer. The first parameter used in <c>SmartDashboard.putData</c>. Not used for display.
     * @param name The name of the timer. Used for display purposes only.
     * @param duration The duration of the timer.
     */
    public TimerWidget(String name, double duration)
    {
        this(name, duration, Mode.AUTO);
    }

    /**
     * Creates a new TimerWidget.
     * @param id The id of the timer. The first parameter used in <c>SmartDashboard.putData</c>. Not used for display.
     * @param name The name of the timer. Used for display purposes only.
     * @param duration The duration of the timer.
     * @param mode The mode of the timer.
     */
    public TimerWidget(String name, double duration, Mode mode)
    {
        this.name = name;
        this.duration = duration;
        this.mode = mode;
        startTime = LocalTime.now().toNanoOfDay();
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    public void setDuration(double duration)
    {
        this.duration = duration;
    }

    public double getDuration()
    {
        return duration;
    }

    public void setMode(Mode mode)
    {
        this.mode = mode;
    }

    public Mode getMode()
    {
        return mode;
    }

    public long getStartTime()
    {
        return startTime;
    }

    public void reset()
    {
        startTime = LocalTime.now().toNanoOfDay();
    }

    public double getTimeLeft() {
        long nanoInDay = LocalTime.now().toNanoOfDay();
        // idk what you're doing testing robots/widgets at 12am but I ain't judging.
        if (nanoInDay < startTime)
            startTime -= 1000000000L * 86400L; // one billion times 86400 = how many nanoseconds in day
        
        return duration - ((nanoInDay - startTime) / 1000000000.0); // 1 billion nanoseconds in second
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Timer");

        builder.addStringProperty("displayTitle", () -> {
            return TimerWidget.this.name;
        }, null);
        builder.addIntegerProperty("mode", () -> {
            return TimerWidget.this.mode.value;
        }, null);
        builder.addDoubleProperty("duration", () -> {
            return TimerWidget.this.duration;
        }, null);
        builder.addIntegerProperty("startTime", () -> {
            return TimerWidget.this.startTime;
        }, null);
    }
}
