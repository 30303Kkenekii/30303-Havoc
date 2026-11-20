package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class PoseStorage {

    // Default pose coordinates (Used if Auto is not run)
    public static final double DEFAULT_X = 6.625;
    public static final double DEFAULT_Y = 7.25;
    public static final double DEFAULT_HEADING = 0.0;

    // Active stored pose (defaults to 6.625, 7.25, 0.0)
    public static double currentX = DEFAULT_X;
    public static double currentY = DEFAULT_Y;
    public static double currentHeading = DEFAULT_HEADING;

    /**
     * Converts currentX, currentY, and currentHeading into a Pedro Pose
     */
    public static Pose getPose(PoseFactory factory) {
        return factory.of(currentX, currentY, currentHeading);
    }

    /**
     * Saves a Pedro Pose from Auto into storage (auto-converts heading to degrees)
     */
    public static void setPose(Pose pose) {
        currentX = pose.x();
        currentY = pose.y();
        currentHeading = Math.toDegrees(pose.heading());
    }

    /**
     * Resets coordinates back to the default start position
     */
    public static void resetToDefault() {
        currentX = DEFAULT_X;
        currentY = DEFAULT_Y;
        currentHeading = DEFAULT_HEADING;
    }
}