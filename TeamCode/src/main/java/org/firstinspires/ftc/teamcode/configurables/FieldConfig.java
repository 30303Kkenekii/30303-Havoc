package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.opmodes.Alliance.Alliance;

@Config
public class FieldConfig {

    public static final double FIELD_SIZE = 144.0;

    // ==========================================
    // 1. FIELD GOALS & SWITCH LINE (Blue-side coordinates; Red is mirrored)
    // ==========================================
    public static double GOAL_1_X = 83.75;
    public static double GOAL_1_Y = 57.75; // Lower goal: active when Y < Y_SWITCH_LINE

    public static double GOAL_2_X = 83.75;
    public static double GOAL_2_Y = 84.75; // Upper goal: active when Y >= Y_SWITCH_LINE

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
    //    Auto trims live in AutoConfig.
    // ==========================================
    // Goal 1 (Lower, Y < 72)
    public static double TELEOP_GOAL1_TURRET_TRIM = 1;
    public static double TELEOP_GOAL1_RPM_TRIM = -30;

    // Goal 2 (Upper, Y >= 72)
    public static double TELEOP_GOAL2_TURRET_TRIM = 1;
    public static double TELEOP_GOAL2_RPM_TRIM = -30;

    // ==========================================
    // Helpers
    // ==========================================

    /**
     * X of the goal this alliance shoots at. upper=false -> lower goal (Goal 1 side),
     * upper=true -> upper goal (Goal 2 side). Red is the Blue field rotated 180 degrees.
     */
    public static double goalX(Alliance alliance, boolean upper) {
        if (alliance == Alliance.BLUE) {
            return upper ? GOAL_2_X : GOAL_1_X;
        }
        return FIELD_SIZE - (upper ? GOAL_1_X : GOAL_2_X);
    }

    public static double goalY(Alliance alliance, boolean upper) {
        if (alliance == Alliance.BLUE) {
            return upper ? GOAL_2_Y : GOAL_1_Y;
        }
        return FIELD_SIZE - (upper ? GOAL_1_Y : GOAL_2_Y);
    }

    public static Pose getRelocalizePose(Alliance alliance, PoseFactory factory) {
        if (alliance == Alliance.BLUE) {
            return factory.of(BLUE_X, BLUE_Y, BLUE_HEADING_DEG);
        }
        return factory.of(RED_X, RED_Y, RED_HEADING_DEG);
    }
}