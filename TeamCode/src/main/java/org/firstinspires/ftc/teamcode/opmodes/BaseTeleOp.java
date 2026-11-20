package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.FieldConfig;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotSolver;
import org.firstinspires.ftc.teamcode.subsystems.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
public abstract class BaseTeleOp extends LinearOpMode {
    public enum Alliance { BLUE, RED }
    private final Alliance alliance;

    // Runtime active trims
    private double goal1TurretTrim;
    private double goal1TpsTrim;
    private double goal2TurretTrim;
    private double goal2TpsTrim;

    private double distanceToGoal = 0.0;
    private double baseCalculatedTps = 2220.0;

    private Follower follower;
    private PoseFactory poseFactory;

    protected IntakeSubsystem intake;
    protected ShooterSubsystem shooter;
    protected TurretSubsystem turret;
    protected LEDIndicator led;
    protected ShotSolver shotSolver;

    private boolean lastTouchpad = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastRelocalize = false;

    public BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        poseFactory = PoseFactory.degrees();

        // 1. Initialize Pose from PoseStorage (defaults to 6.625, 7.25, 0° if no auto ran)
        Pose startPose = poseFactory.of(
                PoseStorage.currentX,
                PoseStorage.currentY,
                PoseStorage.currentHeading
        );
        follower.setPose(startPose);
        follower.update();
        resetTrimsToDefaults();

        // 2. Initialize Subsystems & Coordinators
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        led = new LEDIndicator(hardwareMap);
        shotSolver = new ShotSolver(intake, shooter);

        TurretConfig.ENABLED = true;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Pose", "X: %.2f, Y: %.2f, H: %.1f°",
                PoseStorage.currentX, PoseStorage.currentY, PoseStorage.currentHeading);
        telemetry.addLine("Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleRelocalize();
            handleTurretAndAim();
            handleDynamicFlywheel();
            handleIntakeAndShooter();
            handleFlower();
            updateLED();

            intake.update(shotSolver.isShooting());
            shooter.update();
            turret.update();
            shotSolver.update();

            sendTelemetry();
        }

        turret.stop();
        shooter.spinDown();
    }

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
        double x = (alliance == Alliance.BLUE) ? FieldConfig.BLUE_X : FieldConfig.RED_X;
        double y = (alliance == Alliance.BLUE) ? FieldConfig.BLUE_Y : FieldConfig.RED_Y;
        double h = (alliance == Alliance.BLUE) ? FieldConfig.BLUE_HEADING_DEG : FieldConfig.RED_HEADING_DEG;

        Pose targetPose = poseFactory.of(x, y, h);
        follower.setPose(targetPose);
        follower.hold(targetPose);
        follower.update();

        PoseStorage.currentX = x;
        PoseStorage.currentY = y;
        PoseStorage.currentHeading = h;

        resetTrimsToDefaults();
    }

    private void resetTrimsToDefaults() {
        goal1TurretTrim = FieldConfig.TELEOP_GOAL1_TURRET_TRIM;
        goal1TpsTrim = FieldConfig.TELEOP_GOAL1_RPM_TRIM;
        goal2TurretTrim = FieldConfig.TELEOP_GOAL2_TURRET_TRIM;
        goal2TpsTrim = FieldConfig.TELEOP_GOAL2_RPM_TRIM;
    }

    private void handleTurretAndAim() {
        Pose currentPose = follower.pose();
        double robotX = currentPose.x();
        double robotY = currentPose.y();
        double robotHeadingDeg = Math.toDegrees(currentPose.heading());

        boolean isUpperGoal = robotY >= FieldConfig.Y_SWITCH_LINE;

        // Alliance-aware goal selection
        double targetGoalX, targetGoalY;
        if (alliance == Alliance.BLUE) {
            targetGoalX = isUpperGoal ? FieldConfig.GOAL_2_X : FieldConfig.GOAL_1_X;
            targetGoalY = isUpperGoal ? FieldConfig.GOAL_2_Y : FieldConfig.GOAL_1_Y;
        } else {
            targetGoalX = isUpperGoal ? (144.0 - FieldConfig.GOAL_1_X) : (144.0 - FieldConfig.GOAL_2_X);
            targetGoalY = isUpperGoal ? (144.0 - FieldConfig.GOAL_1_Y) : (144.0 - FieldConfig.GOAL_2_Y);
        }

        // Bumper aim trimming
        if (gamepad1.right_bumper && !lastRightBumper) {
            if (isUpperGoal) goal2TurretTrim += TurretConfig.TRIM_STEP_DEG;
            else             goal1TurretTrim += TurretConfig.TRIM_STEP_DEG;
        }
        if (gamepad1.left_bumper && !lastLeftBumper) {
            if (isUpperGoal) goal2TurretTrim -= TurretConfig.TRIM_STEP_DEG;
            else             goal1TurretTrim -= TurretConfig.TRIM_STEP_DEG;
        }
        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = gamepad1.left_bumper;

        double activeTurretTrim = isUpperGoal ? goal2TurretTrim : goal1TurretTrim;

        double dx = targetGoalX - robotX;
        double dy = targetGoalY - robotY;
        distanceToGoal = Math.hypot(dx, dy);

        double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));
        double relativeAngle = fieldAngleToGoal - robotHeadingDeg;

        while (relativeAngle > 180.0) relativeAngle -= 360.0;
        while (relativeAngle < -180.0) relativeAngle += 360.0;

        double turretTargetAngle = -relativeAngle + activeTurretTrim;
        turret.setTargetAngle(turretTargetAngle);
    }

    private void handleDynamicFlywheel() {
        if (gamepad1.right_stick_button) shooter.spinUp();
        if (gamepad1.left_stick_button)  shooter.spinDown();

        boolean isUpperGoal = follower.pose().y() >= FieldConfig.Y_SWITCH_LINE;

        if (gamepad1.dpad_up && !lastDpadUp) {
            if (isUpperGoal) goal2TpsTrim += ShooterSubsystem.TRIM_STEP;
            else             goal1TpsTrim += ShooterSubsystem.TRIM_STEP;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            if (isUpperGoal) goal2TpsTrim -= ShooterSubsystem.TRIM_STEP;
            else             goal1TpsTrim -= ShooterSubsystem.TRIM_STEP;
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        baseCalculatedTps = ShooterSubsystem.calculateVelocityFromDistance(distanceToGoal);

        double activeTpsTrim = isUpperGoal ? goal2TpsTrim : goal1TpsTrim;
        double finalTargetTps = Range.clip(baseCalculatedTps + activeTpsTrim, ShooterSubsystem.MIN_TPS, ShooterSubsystem.MAX_TPS);

        shooter.setTargetVelocity(finalTargetTps);
    }

    private void handleIntakeAndShooter() {
        if (gamepad1.dpad_left)  intake.pivotUp();
        if (gamepad1.dpad_right) intake.pivotDown();

        if (gamepad1.a) {
            shotSolver.startRecovery();
        }

        if (gamepad1.right_trigger > 0.3) {
            shotSolver.startShoot();
        }

        if (!shotSolver.isBusy()) {
            if (gamepad1.left_trigger > 0.3) {
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

    private void sendTelemetry() {
        Pose p = follower.pose();
        boolean isUpper = p.y() >= FieldConfig.Y_SWITCH_LINE;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Active Goal", isUpper ? "GOAL 2 (Y >= 72)" : "GOAL 1 (Y < 72)");
        telemetry.addData("Distance to Goal", "%.2f in", distanceToGoal);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Curve Base TPS", "%.0f TPS", baseCalculatedTps);
        telemetry.addData("Active TPS Trim", "%+.0f TPS", isUpper ? goal2TpsTrim : goal1TpsTrim);
        telemetry.addData("Final Target / Actual TPS", "%.0f / %.0f", shooter.getTargetVelocity(), shooter.getCurrentVelocity());

        telemetry.addLine("--------------------------------");
        telemetry.addData("Turret Target / Actual", "%.1f° / %.1f°", turret.getTargetAngle(), turret.getCurrentAngle());
        telemetry.addData("Active Turret Trim", "%+.1f°", isUpper ? goal2TurretTrim : goal1TurretTrim);

        telemetry.addLine("--------------------------------");
        telemetry.addData("Confirmed Full", intake.isConfirmedFull());
        telemetry.addData("Shooting State", shotSolver.getState());
        telemetry.update();
    }
}