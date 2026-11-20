package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class IntakeConfig {
    // --- Servo Presets: Flower Pivot (moves at full servo speed) ---
    public static double FLOWER_UP = .25;
    public static double FLOWER_DOWN = .7;

    // --- Motor Powers ---
    public static double INTAKE_POWER = 1.00;
    public static double SHOOTING_FEED_POWER = .75;
    public static double EJECT_POWER = -1.00;

    // --- Full-Intake Behavior ---
    // When 4 balls are in: stop the intake, and raise the flower if it was down
    // (if it was already up it simply stays up). Turn off to leave the flower alone.
    public static boolean FLOWER_UP_WHEN_FULL = true;

    // --- Auto Beam Timings ---
    public static double BEAM_CONFIRM_SEC = 0.40;
    public static double GATE_OPEN_DELAY_SEC = 0.60;
}