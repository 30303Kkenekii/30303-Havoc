package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShotSolver;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Shot Tuner", group = "Tuners")
public class ShotTuner extends LinearOpMode {

    // ==========================================
    // ALLIANCE & GOAL COORDINATES (Configurable)
    // ==========================================
    public static boolean IS_BLUE = true; // Toggle in Dashboard or with [Back] button

    // Blue Goals
    public static double BLUE_GOAL_1_X = 83.75;
    public static double BLUE_GOAL_1_Y = 57.75;
    public static double BLUE_GOAL_2_X = 83.75;
    public static double BLUE_GOAL_2_Y = 84.75;

    // Red Goals (Mirrored across 144" field)
    public static double RED_GOAL_1_X = 60.25;
    public static double RED_GOAL_1_Y = 86.25;
    public static double RED_GOAL_2_X = 60.25;
    public static double RED_GOAL_2_Y = 59.25;

    // ==========================================
    // TUNER SETTINGS
    // ==========================================
    public static double BASE_VELOCITY_TPS = 2220; // Starting flywheel speed
    public static int COARSE_STEP_TPS = 50;        // D-Pad trim step
    public static double TURRET_TRIM_STEP = 0.5;   // Bumper aim step

    // Subsystems & Followers
    private Follower follower;
    private PoseFactory poseFactory;

    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private LEDIndicator led;
    private ShotSolver shotSolver;

    // Runtime adjustments
    private double currentTargetTps = BASE_VELOCITY_TPS;
    private double currentTurretTrim = 0.0;

    // Data Point Logger (for regression tuning)
    private static class DataPoint {
        double dist, tps, trim;
        DataPoint(double d, double v, double t) { dist = d; tps = v; trim = t; }
    }
    private final List<DataPoint> loggedPoints = new ArrayList<>();

    // Button edge-detection trackers
    private boolean lastBack = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Pedro Pathing at DEFAULT Origin (6.625, 7.25, 0.0°)
        follower = Constants.create(hardwareMap);
        poseFactory = PoseFactory.degrees();
        Pose startPose = poseFactory.of(
                PoseStorage.DEFAULT_X,
                PoseStorage.DEFAULT_Y,
                PoseStorage.DEFAULT_HEADING
        );
        follower.setPose(startPose);
        follower.update();

        // 2. Initialize Subsystems & Coordinators
        intake = new IntakeSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        led = new LEDIndicator(hardwareMap);
        shotSolver = new ShotSolver(intake, shooter);

        TurretConfig.ENABLED = true;
        currentTargetTps = BASE_VELOCITY_TPS;

        telemetry.addLine("=== Shot & Distance Tuner Ready ===");
        telemetry.addLine("Controls:");
        telemetry.addLine(" - Drive: Right Stick (drive/strafe), Left Stick X (turn)");
        telemetry.addLine(" - [Back]: Toggle Alliance (Blue <-> Red)");
        telemetry.addLine(" - D-Pad Up / Down: Target TPS +/- 50");
        telemetry.addLine(" - Bumpers: Turret Trim +/- 0.5°");
        telemetry.addLine(" - Stick Buttons: Flywheel ON / OFF");
        telemetry.addLine(" - Right Trigger: Shoot Test Ball");
        telemetry.addLine(" - [A]: Save Point (Distance vs TPS) for Regression");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // -----------------------------------------------------------
            // 1. Drivetrain Manual Driving (Pedro driveOrHold)
            // -----------------------------------------------------------
            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_x,
                    -gamepad1.left_stick_x
            );
            follower.update();

            // -----------------------------------------------------------
            // 2. Alliance Toggle ([Back] button or Dashboard)
            // -----------------------------------------------------------
            if (gamepad1.back && !lastBack) {
                IS_BLUE = !IS_BLUE;
            }
            lastBack = gamepad1.back;

            // -----------------------------------------------------------
            // 3. Pose & Nearest Goal Math
            // -----------------------------------------------------------
            Pose currentPose = follower.pose();
            double robotX = currentPose.x();
            double robotY = currentPose.y();
            double robotHeadingDeg = Math.toDegrees(currentPose.heading());

            // Active alliance goal coordinates
            double g1X = IS_BLUE ? BLUE_GOAL_1_X : RED_GOAL_1_X;
            double g1Y = IS_BLUE ? BLUE_GOAL_1_Y : RED_GOAL_1_Y;
            double g2X = IS_BLUE ? BLUE_GOAL_2_X : RED_GOAL_2_X;
            double g2Y = IS_BLUE ? BLUE_GOAL_2_Y : RED_GOAL_2_Y;

            // Compute distance to both goals and pick the NEAREST one
            double dist1 = Math.hypot(g1X - robotX, g1Y - robotY);
            double dist2 = Math.hypot(g2X - robotX, g2Y - robotY);

            boolean isGoal1Nearest = dist1 <= dist2;
            double targetGoalX = isGoal1Nearest ? g1X : g2X;
            double targetGoalY = isGoal1Nearest ? g1Y : g2Y;
            double distanceToGoal = isGoal1Nearest ? dist1 : dist2;

            // Vector angle to nearest goal
            double dx = targetGoalX - robotX;
            double dy = targetGoalY - robotY;
            double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

            double relativeAngle = fieldAngleToGoal - robotHeadingDeg;
            while (relativeAngle > 180.0) relativeAngle -= 360.0;
            while (relativeAngle < -180.0) relativeAngle += 360.0;

            // -----------------------------------------------------------
            // 4. Turret Trim Controls (Bumpers)
            // -----------------------------------------------------------
            if (gamepad1.right_bumper && !lastRightBumper) currentTurretTrim += TURRET_TRIM_STEP;
            if (gamepad1.left_bumper && !lastLeftBumper)   currentTurretTrim -= TURRET_TRIM_STEP;
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;

            double turretTargetAngle = -relativeAngle + currentTurretTrim;
            turret.setTargetAngle(turretTargetAngle);

            // -----------------------------------------------------------
            // 5. Flywheel Velocity Controls (D-Pad Up / Down)
            // -----------------------------------------------------------
            if (gamepad1.dpad_up && !lastDpadUp)     currentTargetTps += COARSE_STEP_TPS;
            if (gamepad1.dpad_down && !lastDpadDown) currentTargetTps = Math.max(0, currentTargetTps - COARSE_STEP_TPS);
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            if (gamepad1.right_stick_button) shooter.spinUp();
            if (gamepad1.left_stick_button)  shooter.spinDown();

            shooter.setTargetVelocity(currentTargetTps);

            // -----------------------------------------------------------
            // 6. Shooting & Intake Controls
            // -----------------------------------------------------------
            if (gamepad1.right_trigger > 0.3) shotSolver.startShoot();
            if (gamepad1.left_trigger > 0.3)  intake.eject();
            else if (gamepad1.x)             intake.intake();
            else if (gamepad1.y)             intake.stop();

            if (intake.shouldOpenGate()) shooter.openGate();

            // -----------------------------------------------------------
            // 7. Data Logger (Press [A] to save Distance & TPS)
            // -----------------------------------------------------------
            if (gamepad1.a && !lastA) {
                loggedPoints.add(new DataPoint(distanceToGoal, currentTargetTps, currentTurretTrim));
            }
            lastA = gamepad1.a;

            // -----------------------------------------------------------
            // 8. Run Subsystem Updates
            // -----------------------------------------------------------
            intake.update(shotSolver.isShooting());
            shooter.update();
            turret.update();
            shotSolver.update();

            // -----------------------------------------------------------
            // 9. Telemetry & Live Regression Math
            // -----------------------------------------------------------
            sendTelemetry(robotX, robotY, robotHeadingDeg, distanceToGoal, isGoal1Nearest ? 1 : 2, targetGoalX, targetGoalY);
        }

        turret.stop();
        shooter.spinDown();
    }

    private void sendTelemetry(double rx, double ry, double rh, double dist, int goalNum, double gx, double gy) {
        telemetry.addLine("========================================");
        telemetry.addData(">> DISTANCE TO GOAL <<", "%.2f INCHES", dist);
        telemetry.addData("Target Flywheel TPS", "%.0f TPS  (Actual: %.0f)", currentTargetTps, shooter.getCurrentVelocity());
        telemetry.addData("Turret Aim Trim", "%.1f° (Target: %.1f°, Actual: %.1f°)", currentTurretTrim, turret.getTargetAngle(), turret.getCurrentAngle());
        telemetry.addLine("========================================");

        telemetry.addData("Alliance", IS_BLUE ? "BLUE" : "RED");
        telemetry.addData("Tracking Nearest", "Goal %d at (%.1f, %.1f)", goalNum, gx, gy);
        telemetry.addData("Robot Position", "X: %.2f, Y: %.2f, H: %.1f°", rx, ry, rh);
        telemetry.addData("Flywheel Active", shooter.isFlywheelOn());

        // Live Linear Regression Solver
        if (loggedPoints.size() >= 2) {
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            int n = loggedPoints.size();
            for (DataPoint p : loggedPoints) {
                sumX += p.dist;
                sumY += p.tps;
                sumXY += p.dist * p.tps;
                sumX2 += p.dist * p.dist;
            }
            double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
            double intercept = (sumY - slope * sumX) / n;

            telemetry.addLine("----------------------------------------");
            telemetry.addData("Logged Points", n);
            telemetry.addLine(String.format("Calculated Fit: TPS = (%.2f * Dist) + %.1f", slope, intercept));
        } else {
            telemetry.addLine("----------------------------------------");
            telemetry.addData("Logged Points", "%d / 2 (Press [A] to log current shot)", loggedPoints.size());
        }

        telemetry.update();
    }
}