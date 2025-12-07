package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Collections;
import java.util.List;

public interface Subsystem {
    void init();
    default void update() {}
    default void stop() {}
    default List<String> getTelemetry() {
        return Collections.emptyList();
    }
}
