package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;

@Config
@TeleOp(name = "Turret Dual Zero Finder", group = "Tuners")
public class TurretDualZeroFinder extends LinearOpMode {

    public static double JOG_SPEED = 0.20;

    private CRServo turretLeft;
    private CRServo turretRight;
    private AnalogInput turretLeftAnalog;
    private AnalogInput turretRightAnalog;

    // Calibration scratchpad
    private double vLeft_neg90 = 0.0;
    private double vRight_neg90 = 0.0;
    private double vRight_pos90 = 0.0;
    private double vLeft_180 = 0.0;

    private boolean setNeg90 = false;
    private boolean setPos90 = false;
    private boolean set180 = false;
    private boolean latchedToLeft = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretLeft = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        turretLeft.setPower(0);
        turretRight.setPower(0);

        turretLeftAnalog = hardwareMap.get(AnalogInput.class, "turretLeftAnalog");
        turretRightAnalog = hardwareMap.get(AnalogInput.class, "turretRightAnalog");

        telemetry.addLine("=== High-Precision 180° Chassis Calibrator ===");
        telemetry.addLine("Step 1: Point LEFT (-90°).  Press [X] (Left button)");
        telemetry.addLine("Step 2: Point RIGHT (+90°). Press [B] (Right button)");
        telemetry.addLine("Step 3: Point BACK (-180°).  Press [Y] (Top button)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double stickInput = -gamepad1.left_stick_x;
            double jogPower = stickInput * JOG_SPEED;
            turretLeft.setPower(TurretConfig.INVERT_LEFT ? -jogPower : jogPower);
            turretRight.setPower(TurretConfig.INVERT_RIGHT ? -jogPower : jogPower);

            double vLeft = turretLeftAnalog.getVoltage();
            double vRight = turretRightAnalog.getVoltage();

            // -----------------------------------------------------------
            // Step 1: Left 90° (-90.0°) -> Press [X]
            // -----------------------------------------------------------
            if (gamepad1.x) {
                vLeft_neg90 = vLeft;
                vRight_neg90 = vRight;
                TurretConfig.V_LEFT_AT_NEG90 = vLeft;
                setNeg90 = true;
            }

            // -----------------------------------------------------------
            // Step 2: Right 90° (+90.0°) -> Press [B]
            // Solves 180° Right baseline and auto-calculates Center 0°!
            // -----------------------------------------------------------
            if (gamepad1.b && setNeg90) {
                vRight_pos90 = vRight;
                double deltaVR = vRight_pos90 - vRight_neg90;
                if (Math.abs(deltaVR) > 0.1) {
                    TurretConfig.SCALE_RIGHT = 180.0 / deltaVR;
                    TurretConfig.CENTER_RIGHT = (vRight_pos90 + vRight_neg90) / 2.0;
                    setPos90 = true;
                }
            }

            // -----------------------------------------------------------
            // Step 3: Back 180° (-180.0°) -> Press [Y]
            // Solves Left slope between -90° and -180° (Zero drift!)
            // -----------------------------------------------------------
            if (gamepad1.y && setNeg90) {
                vLeft_180 = vLeft;
                double deltaVL = vLeft_180 - vLeft_neg90;
                if (Math.abs(deltaVL) > 0.1) {
                    TurretConfig.SCALE_LEFT = -90.0 / deltaVL;
                    set180 = true;
                }
            }

            // -----------------------------------------------------------
            // Seamless Live Angle Calculation
            // -----------------------------------------------------------
            double angleFromRight = (vRight - TurretConfig.CENTER_RIGHT) * TurretConfig.SCALE_RIGHT;
            double angleFromLeft = -90.0 + (vLeft - TurretConfig.V_LEFT_AT_NEG90) * TurretConfig.SCALE_LEFT;

            // Latch onto Left when past -135°, return to Right when back past -125°
            if (!latchedToLeft) {
                if (angleFromRight < -135.0) latchedToLeft = true;
            } else {
                if (angleFromLeft > -125.0) latchedToLeft = false;
            }

            double fusedAngle = latchedToLeft ? angleFromLeft : angleFromRight;

            // Set limits with D-Pad
            if (gamepad1.dpad_left)  TurretConfig.MIN_ANGLE_DEG = fusedAngle;
            if (gamepad1.dpad_right) TurretConfig.MAX_ANGLE_DEG = fusedAngle;

            // -----------------------------------------------------------
            // Telemetry Readout
            // -----------------------------------------------------------
            telemetry.addLine("=== CALIBRATION PROGRESS ===");
            telemetry.addData("1. Left (-90°)  [X]", setNeg90 ? "DONE (%.3fV)" : "PRESS X", vLeft_neg90);
            telemetry.addData("2. Right (+90°) [B]", setPos90 ? "DONE (%.3fV)" : "PRESS B", vRight_pos90);
            telemetry.addData("3. Back (-180°) [Y]", set180 ? "DONE (%.3fV)" : "PRESS Y", vLeft_180);

            telemetry.addLine("\n=== LIVE FUSED ANGLE ===");
            telemetry.addData("--> TURRET ANGLE <--", "%.1f°", fusedAngle);
            telemetry.addData("Active Sensor", latchedToLeft ? "LEFT (LATCHED)" : "RIGHT (PRIMARY)");

            if (setNeg90 && setPos90 && set180) {
                telemetry.addLine("\n=== COPY-PASTE TO TurretConfig.java ===");
                telemetry.addLine(String.format("CENTER_RIGHT = %.3f;", TurretConfig.CENTER_RIGHT));
                telemetry.addLine(String.format("SCALE_RIGHT = %.2f;", TurretConfig.SCALE_RIGHT));
                telemetry.addLine(String.format("V_LEFT_AT_NEG90 = %.3f;", TurretConfig.V_LEFT_AT_NEG90));
                telemetry.addLine(String.format("SCALE_LEFT = %.2f;", TurretConfig.SCALE_LEFT));
                telemetry.addLine(String.format("MIN_ANGLE_DEG = %.1f;", TurretConfig.MIN_ANGLE_DEG));
                telemetry.addLine(String.format("MAX_ANGLE_DEG = %.1f;", TurretConfig.MAX_ANGLE_DEG));
            }

            telemetry.update();
        }

        turretLeft.setPower(0);
        turretRight.setPower(0);
    }
}