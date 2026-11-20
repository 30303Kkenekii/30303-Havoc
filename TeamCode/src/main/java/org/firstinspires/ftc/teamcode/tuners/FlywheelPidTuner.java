package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Flywheel PID Tuner", group = "Tuners")
public class FlywheelPidTuner extends LinearOpMode {

    // ==========================================
    // DASHBOARD LIVE TUNABLES
    // ==========================================
    public static double TARGET_VELOCITY = 2220; // Current target TPS
    public static boolean RUN_FLYWHEEL = false;  // Active state

    // Quick Step-Response Presets
    public static double PRESET_X = 2000; // X Button Speed
    public static double PRESET_Y = 2400; // Y Button Speed

    // PIDF Coefficients (Tune in order: F -> P -> D)
    public static double P = 30.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 13.5;

    // Tolerance band for Dashboard graph (visualizing "at speed")
    public static double TOLERANCE_TPS = 30.0;

    // Ball-feed test parameters (Right Trigger)
    public static double TEST_FEED_TIME = 0.50;
    public static double TEST_FEED_POWER = 0.80;
    public static double GATE_OPEN_POS = 0.65;
    public static double GATE_CLOSED_POS = 0.50;

    // ==========================================
    // HARDWARE
    // ==========================================
    private DcMotorEx shooter;
    private Servo transferGate;
    private DcMotor intakeFront, intakeBack;

    private double lastP, lastI, lastD, lastF;
    private boolean lastRightStick = false;
    private boolean lastLeftStick = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private boolean feedingBall = false;
    private final ElapsedTime feedTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Shooter Setup
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // 2. Optional feed hardware (for ball-drop recovery test)
        try {
            transferGate = hardwareMap.get(Servo.class, "transferGate");
            intakeFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
            intakeBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");
            intakeFront.setDirection(DcMotor.Direction.REVERSE);
            transferGate.setPosition(GATE_CLOSED_POS);
        } catch (Exception ignored) {
            // Continues if only the shooter is plugged in
        }

        applyPidCoefficients();

        telemetry.addLine("Flywheel PID Tuner Ready.");
        telemetry.addLine("Step Test Presets:");
        telemetry.addLine(" - X: Target 2000 TPS");
        telemetry.addLine(" - Y: Target 2400 TPS");
        telemetry.addLine(" - B: Target 0 TPS (Stop)");
        telemetry.addLine("Manual Trims:");
        telemetry.addLine(" - D-Pad Up/Down: Target +/- 50 TPS");
        telemetry.addLine(" - Right/Left Stick: Manual ON / OFF");
        telemetry.addLine(" - Right Trigger: Test Feed a Ball");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleGamepad();

            // Re-apply PIDF gains only when changed in Dashboard
            if (P != lastP || I != lastI || D != lastD || F != lastF) {
                applyPidCoefficients();
            }

            // Command Velocity
            if (RUN_FLYWHEEL && TARGET_VELOCITY > 0) {
                shooter.setVelocity(TARGET_VELOCITY);
            } else {
                shooter.setVelocity(0);
            }

            handleTestFeed();
            sendGraphTelemetry();
        }

        // Safe shutdown on stop
        shooter.setVelocity(0);
        if (transferGate != null) transferGate.setPosition(GATE_CLOSED_POS);
        if (intakeFront != null) {
            intakeFront.setPower(0);
            intakeBack.setPower(0);
        }
    }

    private void handleGamepad() {
        // --- Quick Presets for Step Response ---
        if (gamepad1.x) {
            TARGET_VELOCITY = PRESET_X;
            RUN_FLYWHEEL = true;
        }
        if (gamepad1.y) {
            TARGET_VELOCITY = PRESET_Y;
            RUN_FLYWHEEL = true;
        }
        if (gamepad1.b) {
            TARGET_VELOCITY = 0;
            RUN_FLYWHEEL = false;
        }

        // Stick Click Overrides
        if (gamepad1.right_stick_button && !lastRightStick) RUN_FLYWHEEL = true;
        if (gamepad1.left_stick_button && !lastLeftStick)   RUN_FLYWHEEL = false;
        lastRightStick = gamepad1.right_stick_button;
        lastLeftStick = gamepad1.left_stick_button;

        // D-Pad Coarse Adjust (+/- 50 TPS)
        if (gamepad1.dpad_up && !lastDpadUp)     TARGET_VELOCITY += 50;
        if (gamepad1.dpad_down && !lastDpadDown) TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - 50);
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // Trigger ball-feed recovery test
        if (gamepad1.right_trigger > 0.3 && !feedingBall) {
            feedingBall = true;
            feedTimer.reset();
        }
    }

    private void handleTestFeed() {
        if (!feedingBall) return;

        if (transferGate != null && intakeFront != null) {
            transferGate.setPosition(GATE_OPEN_POS);
            intakeFront.setPower(TEST_FEED_POWER);
            intakeBack.setPower(TEST_FEED_POWER);
        }

        if (feedTimer.seconds() >= TEST_FEED_TIME) {
            feedingBall = false;
            if (transferGate != null && intakeFront != null) {
                transferGate.setPosition(GATE_CLOSED_POS);
                intakeFront.setPower(0);
                intakeBack.setPower(0);
            }
        }
    }

    private void applyPidCoefficients() {
        shooter.setVelocityPIDFCoefficients(P, I, D, F);
        lastP = P;
        lastI = I;
        lastD = D;
        lastF = F;
    }

    private void sendGraphTelemetry() {
        double currentVel = shooter.getVelocity();
        double activeTarget = (RUN_FLYWHEEL && TARGET_VELOCITY > 0) ? TARGET_VELOCITY : 0;
        double error = activeTarget - currentVel;

        // Formatted for FTC Dashboard's Graphing tab
        telemetry.addData("target", activeTarget);
        telemetry.addData("actual", currentVel);
        telemetry.addData("error", error);
        telemetry.addData("upperTol", (activeTarget > 0) ? activeTarget + TOLERANCE_TPS : 0);
        telemetry.addData("lowerTol", (activeTarget > 0) ? activeTarget - TOLERANCE_TPS : 0);

        telemetry.addData("RUNNING", RUN_FLYWHEEL);
        telemetry.addData("TARGET_VELOCITY", TARGET_VELOCITY);
        telemetry.addData("P", P);
        telemetry.addData("I", I);
        telemetry.addData("D", D);
        telemetry.addData("F", F);
        telemetry.update();
    }
}