package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Velocity Test")
public class UnloadedMotorVelocityTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotorEx shooterMotor = hardwareMap.get(
                DcMotorEx.class, "shooter");

        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Shooter Velocity Test Ready");
        telemetry.addLine("Motor will run at 25% power.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        int lastPosition = shooterMotor.getCurrentPosition();
        long lastTime = System.nanoTime();

        double customVelocity = 0;
        double regularVelocity = 0;

        shooterMotor.setPower(0.25);

        try {
            while (opModeIsActive()) {

                // Current encoder position
                int position = shooterMotor.getCurrentPosition();

                // Current timestamp
                long now = System.nanoTime();

                // Time difference in seconds
                double dt = (now - lastTime) / 1e9;

                // CUSTOM VELOCITY
                if (dt > 0) {
                    customVelocity =
                            (position - lastPosition) / dt;
                }

                // REGULAR FTC SDK VELOCITY
                regularVelocity = shooterMotor.getVelocity();

                // Update previous measurements
                lastPosition = position;
                lastTime = now;

                // TELEMETRY
                telemetry.addData(
                        "Motor Power", "%.2f",
                        shooterMotor.getPower());

                telemetry.addData(
                        "Encoder Position", position);

                telemetry.addData(
                        "Delta Time (ms)", "%.3f",
                        dt * 1000);

                telemetry.addLine("------------------------");

                telemetry.addData(
                        "Regular Velocity (ticks/s)", "%.2f",
                        regularVelocity);

                telemetry.addData(
                        "Custom Velocity (ticks/s)", "%.2f",
                        customVelocity);

                telemetry.addData(
                        "Difference (ticks/s)", "%.2f",
                        customVelocity - regularVelocity);

                telemetry.update();

                sleep(20);
            }
        } finally {
            shooterMotor.setPower(0);
        }
    }
}