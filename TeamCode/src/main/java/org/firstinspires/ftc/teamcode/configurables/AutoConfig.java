package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmodes.Alliance.Alliance;

/**
 * Every Dashboard-tunable value for Autonomous lives here.
 *
 * "Goal 1" = LOWER goal (Y < Y_SWITCH_LINE), "Goal 2" = UPPER goal (Y >= Y_SWITCH_LINE),
 * for both alliances. Auto always aims at whichever of the two goals is nearest the robot,
 * then applies that alliance + goal's own turret trim and RPM trim.
 */
@Config
public class AutoConfig {

    // ==========================================
    // 1. TIMINGS
    // ==========================================
    public static double DELAY_BEFORE_FIRE = 1.0;   // Settle time after arriving, before feeding
    public static double SHOOT_TIME = 0.60;         // How long the feed runs per volley
    public static double RPM_TOLERANCE = 20.0;      // "At speed" window (TPS)
    public static double MAX_SPINUP_WAIT = 2.0;     // Extra wait before feeding anyway

    public static double INTAKE_TIMEOUT_1 = 3.5;    // First intake leg
    public static double INTAKE_TIMEOUT_2 = 3.5;    // Second intake leg

    public static double MAX_PATH_SPEED = 1.00;     // Foresight max path speed (1.00 = 100%)

    // ==========================================
    // 2. BLUE ALLIANCE TRIMS
    // ==========================================
    public static double BLUE_GOAL1_TURRET_TRIM = 0.0;   // degrees
    public static double BLUE_GOAL1_RPM_TRIM = -100;     // TPS added to the curve
    public static double BLUE_GOAL2_TURRET_TRIM = 0.0;
    public static double BLUE_GOAL2_RPM_TRIM = -100;

    // ==========================================
    // 3. RED ALLIANCE TRIMS
    // (RPM defaults are the old "-100 general trim + 70 red-only trim" = -30)
    // ==========================================
    public static double RED_GOAL1_TURRET_TRIM = 0.0;
    public static double RED_GOAL1_RPM_TRIM = -30;
    public static double RED_GOAL2_TURRET_TRIM = 0.0;
    public static double RED_GOAL2_RPM_TRIM = -30;

    // ==========================================
    // Lookup helpers
    // ==========================================
    public static double turretTrim(Alliance alliance, boolean goal1) {
        if (alliance == Alliance.BLUE) {
            return goal1 ? BLUE_GOAL1_TURRET_TRIM : BLUE_GOAL2_TURRET_TRIM;
        }
        return goal1 ? RED_GOAL1_TURRET_TRIM : RED_GOAL2_TURRET_TRIM;
    }

    public static double rpmTrim(Alliance alliance, boolean goal1) {
        if (alliance == Alliance.BLUE) {
            return goal1 ? BLUE_GOAL1_RPM_TRIM : BLUE_GOAL2_RPM_TRIM;
        }
        return goal1 ? RED_GOAL1_RPM_TRIM : RED_GOAL2_RPM_TRIM;
    }
}