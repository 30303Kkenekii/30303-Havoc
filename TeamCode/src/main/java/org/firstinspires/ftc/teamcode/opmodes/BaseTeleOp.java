package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.opmodes.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.configurables.FieldConfig;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfig;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class BaseTeleOp extends LinearOpMode {
    private static final double TRIGGER_THRESHOLD = 0.3;

    private final Alliance alliance;

    // Runtime trims (loaded from FieldConfig on boot and on relocalize).
    // Goal 1 = lower goal (Y < Y_SWITCH_LINE), Goal 2 = upper goal (Y >= Y_SWITCH_LINE).
    private double goal1TurretTrim;
    private double goal1RpmTrim;
    private double goal2TurretTrim;
    private double goal2RpmTrim;

    // Goal tracking (refreshed once per loop)
    private boolean upperGoal = false;
    private double goalX = 0.0;
    private double goalY = 0.0;
    private double distanceToGoal = 0.0;
    private double baseCalculatedTps = 0.0;

    private Follower follower;
    private PoseFactory poseFactory;

    // Subsystems are created AFTER Start so nothing moves during init.
    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected TurretSubsystem turret;
    protected LEDIndicator led;
    protected ShotSolver shotSolver;
    private boolean wasAtSpeed = false;
    private boolean lastTouchpad = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastRelocalize = false;
    private boolean lastSotmToggle = false;

    public BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---------------------------------------------------------------
        // INIT: only set up the localizer/pose. No mechanism is commanded,
        // so nothing moves while we wait for Start.
        // ---------------------------------------------------------------
        follower = Constants.create(hardwareMap);
        poseFactory = PoseFactory.degrees();

        // Pose carried over from Auto (defaults to the start pose if no auto ran)
        follower.setPose(PoseStorage.getPose(poseFactory));
        follower.update();
        resetTrimsToDefaults();

        TurretConfig.ENABLED = true;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, H: %.1f°",
                PoseStorage.currentX, PoseStorage.currentY, PoseStorage.currentHeading);
        telemetry.addLine("Initialized. Nothing will move until Start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------------------------------------------
        // START: bring every mechanism to its init state (the subsystem
        // constructors do this: flower up, transfer gate closed, LED idle,
        // turret/drive stopped), then spin up the flywheel.
        // ---------------------------------------------------------------
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        led = new LEDIndicator(hardwareMap);
        shotSolver = new ShotSolver(intake, shooter);

        shooter.spinUp();

        // Don't let a held Start/Back button at the beginning trigger a relocalize
        lastRelocalize = gamepad1.start || gamepad1.back;
        lastSotmToggle = gamepad1.left_stick_button;

        while (opModeIsActive()) {
            handleDrive();
            handleRelocalize();
            updateGoalTracking();
            handleTurretAndAim();
            handleDynamicFlywheel();
            handleIntakeAndShooter();
            handleFlower();
            updateLED();
            updateRumble();

            intake.update(shotSolver.isShooting());
            shooter.update();
            turret.update();
            shotSolver.update();

            sendTelemetry();
        }

        // Make sure everything is actually commanded off on exit
        gamepad1.stopRumble();
        intake.stop();
        turret.stop();
        shooter.spinDown();
        shooter.update();
    }

    // ==========================================
    // DRIVE & POSE
    // ==========================================
    private void handleDrive() {
        ManualDrive.driveOrHold(
                follower,
                -gamepad1.right_stick_y,
                -gamepad1.right_stick_x,
                -gamepad1.left_stick_x
        );
        follower.update();
    }

    private void handleRelocalize() {
        boolean relocalizePressed = gamepad1.start || gamepad1.back;
        if (relocalizePressed && !lastRelocalize) {
            relocalize();
        }
        lastRelocalize = relocalizePressed;
    }

    private void relocalize() {
        Pose targetPose = FieldConfig.getRelocalizePose(alliance, poseFactory);
        follower.setPose(targetPose);
        follower.hold(targetPose);
        follower.update();

        PoseStorage.setPose(targetPose);
        resetTrimsToDefaults();
    }

    // ==========================================
    // GOAL SELECTION & TRIMS
    // ==========================================
    private void resetTrimsToDefaults() {
        goal1TurretTrim = FieldConfig.TELEOP_GOAL1_TURRET_TRIM;
        goal1RpmTrim = FieldConfig.TELEOP_GOAL1_RPM_TRIM;
        goal2TurretTrim = FieldConfig.TELEOP_GOAL2_TURRET_TRIM;
        goal2RpmTrim = FieldConfig.TELEOP_GOAL2_RPM_TRIM;
    }

    private double activeTurretTrim() { return upperGoal ? goal2TurretTrim : goal1TurretTrim; }
    private double activeRpmTrim()    { return upperGoal ? goal2RpmTrim : goal1RpmTrim; }

    private void adjustTurretTrim(double delta) {
        if (upperGoal) goal2TurretTrim += delta; else goal1TurretTrim += delta;
    }

    private void adjustRpmTrim(double delta) {
        if (upperGoal) goal2RpmTrim += delta; else goal1RpmTrim += delta;
    }

    /** Picks the goal for the robot's current side of the field and measures the distance to it. */
    private void updateGoalTracking() {
        Pose pose = follower.pose();
        upperGoal = pose.y() >= FieldConfig.Y_SWITCH_LINE;
        goalX = FieldConfig.goalX(alliance, upperGoal);
        goalY = FieldConfig.goalY(alliance, upperGoal);
        distanceToGoal = Math.hypot(goalX - pose.x(), goalY - pose.y());
    }

    // ==========================================
    // TURRET & FLYWHEEL
    // ==========================================
    private void handleTurretAndAim() {
        // Bumper aim trimming
        if (gamepad1.right_bumper && !lastRightBumper) adjustTurretTrim(TurretConfig.TRIM_STEP_DEG);
        if (gamepad1.left_bumper && !lastLeftBumper)   adjustTurretTrim(-TurretConfig.TRIM_STEP_DEG);
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        // Shoot-on-the-move toggle (Left Stick Click)
        boolean sotmToggle = gamepad1.left_stick_button;
        if (sotmToggle && !lastSotmToggle) turret.toggleSotm();
        lastSotmToggle = sotmToggle;

        // The turret handles goal aiming and (when enabled) the shoot-on-the-move lead.
        // The flywheel still uses the real distance to the goal.
        Pose pose = follower.pose();
        Velocity vel = follower.velocity();
        turret.aimAt(pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                vel.vx, vel.vy, goalX, goalY, activeTurretTrim());
    }

    private void handleDynamicFlywheel() {
        if (gamepad1.dpad_right) shooter.spinUp();
        if (gamepad1.dpad_left)  shooter.spinDown();

        if (gamepad1.dpad_up && !lastDpadUp)     adjustRpmTrim(ShooterConfig.TRIM_STEP);
        if (gamepad1.dpad_down && !lastDpadDown) adjustRpmTrim(-ShooterConfig.TRIM_STEP);
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        baseCalculatedTps = ShooterSubsystem.calculateVelocityFromDistance(distanceToGoal);
        double finalTargetTps = Range.clip(baseCalculatedTps + activeRpmTrim(),
                ShooterConfig.MIN_TPS, ShooterConfig.MAX_TPS);

        shooter.setTargetVelocity(finalTargetTps);
    }

    // ==========================================
    // INTAKE, SHOOTING & FLOWER
    // ==========================================
    private void handleIntakeAndShooter() {
        if (gamepad1.a) {
            shotSolver.startRecovery();
        }

        if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            shotSolver.startShoot();
        }

        if (!shotSolver.isBusy()) {
            if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                intake.eject();
            } else if (gamepad1.x) {
                intake.intake();
            } else if (gamepad1.y) {
                intake.stop();
            } else if (gamepad1.b) {
                intake.intakeShooting();
            }

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

    // ==========================================
    // LED & TELEMETRY
    // ==========================================
    private void updateLED() {
        if (shotSolver.isShooting()) {
            led.setShooting();
        } else if (intake.isConfirmedFull()) {
            led.setFull();
        } else if (intake.isIntaking()) {
            led.blinkAlliance(alliance);
        } else {
            led.setIdle();
        }
    }
    private void updateRumble() {
        double error = Math.abs(shooter.getTargetVelocity() - shooter.getCurrentVelocity());
        boolean isAtSpeed = shooter.isFlywheelOn() && error <= 50.0;

        if (isAtSpeed && !wasAtSpeed) {
            // 1.0, 1.0 = full strength on both left/right motors
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else if (!isAtSpeed && wasAtSpeed) {
            gamepad1.stopRumble();
        }

        wasAtSpeed = isAtSpeed;
    }
    private void sendTelemetry() {
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Active Goal", upperGoal ? "GOAL 2 (Y >= 72)" : "GOAL 1 (Y < 72)");
        telemetry.addData("Distance to Goal", "%.2f in", distanceToGoal);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Flywheel", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Curve Base TPS", "%.0f TPS", baseCalculatedTps);
        telemetry.addData("Active TPS Trim", "%+.0f TPS", activeRpmTrim());
        telemetry.addData("Final Target / Actual TPS", "%.0f / %.0f", shooter.getTargetVelocity(), shooter.getCurrentVelocity());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Turret Target / Actual", "%.1f° / %.1f°", turret.getTargetAngle(), turret.getCurrentAngle());
        telemetry.addData("Active Turret Trim", "%+.1f°", activeTurretTrim());
        telemetry.addData("SOTM", turret.isSotmEnabled() ? "ON" : "OFF");
        if (turret.isSotmEnabled()) {
            telemetry.addData("SOTM Lead / Aim Shift", "%.3f s / %.1f in", turret.getSotmLeadSec(), turret.getSotmShiftIn());
        }

        telemetry.addLine("--------------------------------");
        telemetry.addData("Flower", intake.isFlowerDown() ? "DOWN" : "UP");
        telemetry.addData("Confirmed Full", intake.isConfirmedFull());
        telemetry.addData("Shooting State", shotSolver.getState());
        telemetry.update();
    }
}