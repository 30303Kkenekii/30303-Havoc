package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "Turret PID Tuner", group = "Tuners")
public class TurretPidTuner extends LinearOpMode {

    public static double TARGET_ANGLE = 0.0;

    // Quick Step-Response Presets for testing
    public static double PRESET_A = 0.0;   // Center
    public static double PRESET_X = -45.0; // Left step
    public static double PRESET_Y = 45.0;  // Right step

    private TurretSubsystem turret;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turret = new TurretSubsystem(hardwareMap);

        telemetry.addLine("=== Turret PID Tuner Ready ===");
        telemetry.addLine("Controls:");
        telemetry.addLine(" - [A]: Target 0° (Center)");
        telemetry.addLine(" - [X]: Target -45° (Left Step)");
        telemetry.addLine(" - [Y]: Target +45° (Right Step)");
        telemetry.addLine(" - [B]: Emergency Stop (Disable Power)");
        telemetry.addLine(" - D-Pad Up / Down: Target +/- 5°");
        telemetry.addLine(" - Right Stick Button: Enable Turret");
        telemetry.addLine(" - Left Stick Button:  Disable Turret");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleGamepad();

            turret.setTargetAngle(TARGET_ANGLE);
            turret.update();

            // Formatted for FTC Dashboard Graph tab
            telemetry.addData("target", TARGET_ANGLE);
            telemetry.addData("actual", turret.getCurrentAngle());
            telemetry.addData("error", TARGET_ANGLE - turret.getCurrentAngle());
            telemetry.addData("power", turret.getPower());
            telemetry.addData("Active Sensor", turret.isLatchedToLeft() ? "LEFT (LATCHED)" : "RIGHT (PRIMARY)");
            telemetry.addData("ENABLED", TurretConfig.ENABLED);

            telemetry.addLine("--------------------------------");
            telemetry.addData("kP", TurretConfig.kP);
            telemetry.addData("kI", TurretConfig.kI);
            telemetry.addData("kD", TurretConfig.kD);
            telemetry.addData("kS", TurretConfig.kS);
            telemetry.addData("MAX_POWER", TurretConfig.MAX_POWER);

            telemetry.update();
        }

        turret.stop();
    }

    private void handleGamepad() {
        // Quick step setpoints
        if (gamepad1.a) { TARGET_ANGLE = PRESET_A; TurretConfig.ENABLED = true; }
        if (gamepad1.x) { TARGET_ANGLE = PRESET_X; TurretConfig.ENABLED = true; }
        if (gamepad1.y) { TARGET_ANGLE = PRESET_Y; TurretConfig.ENABLED = true; }
        if (gamepad1.b) { TurretConfig.ENABLED = false; } // Quick disable / e-stop

        // Stick Click enable / disable
        if (gamepad1.right_stick_button) TurretConfig.ENABLED = true;
        if (gamepad1.left_stick_button)  TurretConfig.ENABLED = false;

        // D-Pad trim (+/- 5 degrees)
        if (gamepad1.dpad_up && !lastDpadUp)     TARGET_ANGLE += 5.0;
        if (gamepad1.dpad_down && !lastDpadDown) TARGET_ANGLE -= 5.0;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }
}