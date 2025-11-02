package org.firstinspires.ftc.teamcode;

// Settings class for global robot constants
public final class Settings {

    private Settings() {
        throw new UnsupportedOperationException("Utility class - cannot be instantiated");
    }

    /** Deadzone threshold for joystick or controller inputs */
    public static double DEADZONE_THRESHOLD = 0.1; // Not final so FTC Dashboard can modify it

    /** Contains constants for autonomous mode */
    public static final class Autonomous {

        private Autonomous() {
            throw new UnsupportedOperationException("Utility class - cannot be instantiated");
        }

        // Encoder and motion parameters
        public static final double TICKS_PER_REVOLUTION = 537.6;
        public static final double WHEEL_DIAMETER_IN = 3.77952;
        public static final double TICKS_PER_IN = TICKS_PER_REVOLUTION / (WHEEL_DIAMETER_IN * Math.PI);

        // Default drive settings (modifiable)
        public static int DEFAULT_DRIVE_TIMEOUT_MS = 3000;
        public static double DEFAULT_DRIVE_MAX_POWER = 0.5;
        public static double DEFAULT_DRIVE_MIN_POWER = 0.1;

        public static int DEFAULT_TURN_TIMEOUT_MS = 2000;
        public static double DEFAULT_TURN_MAX_POWER = 0.5;
        public static double DEFAULT_TURN_MIN_POWER = 0.1;

        /** PID constants for driving control */
        public static final class DrivePID {
            public static double kP = 1.5;
            public static double EPSILON = 3.0;
        }

        /** PID constants for turning control */
        public static final class TurnPID {
            public static double kP = 1.5;
            public static double EPSILON = 3.0;
        }
    }
}
