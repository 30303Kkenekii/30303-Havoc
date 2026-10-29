package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
public abstract class BaseTeleOp extends LinearOpMode {
    public enum Alliance { BLUE, RED }
    private final Alliance alliance;

    // ==========================================
    // SHOOT CONFIGS (Updated Values)
    // ==========================================
    public static double SHOOT_FEED_POWER = 0.80;  // 80% Power feeding flywheel
    public static double SHOOT_FEED_TIME = 0.60;   // 0.60s feed duration
    public static double SHOOT_LOWER_DELAY = 0.20; // Delay before lowering intake pivot

    // ==========================================
    // FALSE-POSITIVE RECOVERY CONFIG
    // ==========================================
    public static double RECOVERY_DELAY_SEC = 0.60; // Wait gate delay before lowering & resuming

    private enum ShootState { IDLE, FEEDING, WAITING_TO_LOWER }
    private ShootState shootState = ShootState.IDLE;
    private final ElapsedTime shootTimer = new ElapsedTime();

    // False-positive recovery state
    private boolean isRecovering = false;
    private final ElapsedTime recoveryTimer = new ElapsedTime();

    private DcMotor frontLeft, frontRight, rearLeft, rearRight;

    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected LEDIndicator led;

    private boolean lastTouchpad = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    public BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        led = new LEDIndicator(hardwareMap);

        telemetry.addData("Alliance", alliance);
        telemetry.addLine("Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleFlywheel();
            handleTurretPlaceholders();
            handleIntakeAndShooterSequence();
            handleFlower();
            updateLED();

            intake.update(isShooting());
            shooter.update();

            sendTelemetry();
        }
    }

    private void handleDrive() {
        float forward = gamepad1.right_stick_y;
        float strafe = -gamepad1.right_stick_x;
        float rotate = gamepad1.left_stick_x;

        frontLeft.setPower(forward + (strafe - rotate));
        frontRight.setPower(forward - (strafe - rotate));
        rearLeft.setPower(forward - (strafe + rotate));
        rearRight.setPower(forward + (strafe + rotate));
    }

    private void handleFlywheel() {
        if (gamepad1.right_stick_button) shooter.spinUp();
        if (gamepad1.left_stick_button)  shooter.spinDown();

        if (gamepad1.dpad_up && !lastDpadUp) {
            shooter.trimTargetVelocity(ShooterSubsystem.TRIM_STEP);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooter.trimTargetVelocity(-ShooterSubsystem.TRIM_STEP);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
    }

    private void handleTurretPlaceholders() {
        // Bumpers reserved for Turret
    }

    private void handleIntakeAndShooterSequence() {
        // Manual Pivot: Left = UP, Right = DOWN
        if (gamepad1.dpad_left)  intake.pivotUp();
        if (gamepad1.dpad_right) intake.pivotDown();

        // -----------------------------------------------------------
        // 1. False-Positive Recovery Trigger (Button A)
        // -----------------------------------------------------------
        if (gamepad1.a && !isShooting()) {
            shooter.closeGate();         // 1. Immediately close gate
            isRecovering = true;
            recoveryTimer.reset();
        }

        // Recovery State Machine (Non-blocking)
        if (isRecovering) {
            if (recoveryTimer.seconds() >= RECOVERY_DELAY_SEC) {
                intake.pivotDown();          // 2. Lower intake to floor
                intake.intake();             // 3. Resume full intaking
                intake.resetAutoSequence();  // 4. Re-arm beam break detection
                isRecovering = false;
            }
        }

        // -----------------------------------------------------------
        // 2. Right Trigger Starts Shoot Sequence
        // -----------------------------------------------------------
        if (gamepad1.right_trigger > 0.3 && shootState == ShootState.IDLE) {
            isRecovering = false; // Cancel any active recovery
            shootState = ShootState.FEEDING;
            shootTimer.reset();
            intake.setPower(SHOOT_FEED_POWER);
            shooter.openGate();
        }

        // 3. Shoot State Machine
        if (shootState == ShootState.FEEDING) {
            intake.setPower(SHOOT_FEED_POWER);
            if (shootTimer.seconds() >= SHOOT_FEED_TIME) {
                intake.stop();
                shooter.closeGate();
                shootState = ShootState.WAITING_TO_LOWER;
                shootTimer.reset();
            }
        } else if (shootState == ShootState.WAITING_TO_LOWER) {
            intake.stop();
            if (shootTimer.seconds() >= SHOOT_LOWER_DELAY) {
                intake.pivotDown();
                intake.intake();
                intake.resetAutoSequence();
                shootState = ShootState.IDLE;
            }
        }

        // 4. Manual Intake Controls (When NOT shooting and NOT recovering)
        if (shootState == ShootState.IDLE && !isRecovering) {
            if (gamepad1.left_trigger > 0.3) {
                intake.eject();
            } else if (gamepad1.x) {
                intake.intake();
            } else if (gamepad1.y) {
                intake.stop();
            } else if (gamepad1.b) {
                intake.setPower(SHOOT_FEED_POWER);
            }

            // Open gate if auto-beam sequence triggered it
            if (intake.shouldOpenGate()) {
                shooter.openGate();
            }
        }
    }

    private void handleFlower() {
        boolean touchpad = gamepad1.touchpad;
        if (touchpad && !lastTouchpad) {
            intake.toggleFlower();
        }
        lastTouchpad = touchpad;
    }

    private boolean isShooting() {
        return shootState != ShootState.IDLE;
    }

    private void updateLED() {
        if (isShooting()) {
            led.setShooting();
        } else if (intake.isConfirmedFull()) {
            led.setFull();
        } else if (intake.isIntaking()) {
            led.blinkAlliance(alliance);
        } else {
            led.setIdle();
        }
    }

    private void sendTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Flywheel On", shooter.isFlywheelOn());
        telemetry.addData("Flywheel Target TPS", shooter.getTargetVelocity());
        telemetry.addData("Flywheel Actual TPS", shooter.getCurrentVelocity());
        telemetry.addData("Gate Open", shooter.isGateOpen());
        telemetry.addData("Intake Cmd Power", intake.getPower());
        telemetry.addData("Beam Broken (Raw)", intake.isBeamBroken());
        telemetry.addData("Confirmed Full", intake.isConfirmedFull());
        telemetry.addData("Shooting State", shootState);
        telemetry.addData("Recovering (A Button)", isRecovering);
        telemetry.update();
    }
}