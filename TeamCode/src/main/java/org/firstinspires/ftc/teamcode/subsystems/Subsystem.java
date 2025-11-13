package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem {
    void init();
    default void update() {}
    default void stop() {}
    default String[] getTelemetry() {
        return new String[0];
    }
}
