package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

@Config
public class FieldConfig {

    // ==========================================
    // 1. FIELD GOALS & SWITCH LINE
    // ==========================================
    public static double GOAL_1_X = 83.75;
    public static double GOAL_1_Y = 57.75; // Active when Y < Y_SWITCH_LINE

    public static double GOAL_2_X = 83.75;
    public static double GOAL_2_Y = 84.75; // Active when Y >= Y_SWITCH_LINE

    public static double Y_SWITCH_LINE = 72.0;

    // ==========================================
    // 2. ALLIANCE RELOCALIZE / START POSES
    // ==========================================
    public static double BLUE_X = 84.0;
    public static double BLUE_Y = 6.3;
    public static double BLUE_HEADING_DEG = 90;

    public static double RED_X = 56.6;
    public static double RED_Y = 133.6;
    public static double RED_HEADING_DEG = 270;

    // ==========================================
    // 3. BASELINE TELEOP TRIMS (Loaded on boot/relocalize)
    // ==========================================
    // Goal 1 (Lower, Y < 72)
    public static double TELEOP_GOAL1_TURRET_TRIM = 1;
    public static double TELEOP_GOAL1_RPM_TRIM = -30;

    // Goal 2 (Upper, Y >= 72)
    public static double TELEOP_GOAL2_TURRET_TRIM = 1;
    public static double TELEOP_GOAL2_RPM_TRIM = -30;

    // ==========================================
    // 4. BASELINE AUTO TRIMS (For Autonomous routines)
    // ==========================================
    // Goal 1 (Lower, Y < 72)
    public static double AUTO_GOAL1_TURRET_TRIM = 0.0;
    public static double AUTO_GOAL1_RPM_TRIM = -100;

    // Goal 2 (Upper, Y >= 72)
    public static double AUTO_GOAL2_TURRET_TRIM = 0.0;
    public static double AUTO_GOAL2_RPM_TRIM = -100;

    // --- Helper Methods ---
    public static Pose getBlueRelocalizePose(PoseFactory factory) {
        return factory.of(BLUE_X, BLUE_Y, BLUE_HEADING_DEG);
    }

    public static Pose getRedRelocalizePose(PoseFactory factory) {
        return factory.of(RED_X, RED_Y, RED_HEADING_DEG);
    }
}