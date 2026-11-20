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
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@Config
@TeleOp(name = "Turret Goal Tracker Test", group = "Test")
public class TurretGoalTrackerTest extends LinearOpMode {

    // ==========================================
    // FIELD GOALS & SWITCH LINE
    // ==========================================
    public static double GOAL_1_X = 83.75;
    public static double GOAL_1_Y = 57.75; // Active when Y < 72.0

    public static double GOAL_2_X = 83.75;
    public static double GOAL_2_Y = 84.75; // Active when Y >= 72.0

    public static double Y_SWITCH_LINE = 72.0;

    // Start Pose
    public static double START_X = 6.625;
    public static double START_Y = 7.25;
    public static double START_HEADING_DEG = 0.0;

    // Driver trim offset (bumpers allow fine-tuning aim on the fly)
    public static double manualTrimDeg = 0.0;

    private Follower follower;
    private TurretSubsystem turret;

    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);
        PoseFactory poseFactory = PoseFactory.degrees();
        Pose startPose = poseFactory.of(START_X, START_Y, START_HEADING_DEG);
        follower.setPose(startPose);
        follower.update();

        // 2. Initialize Turret
        turret = new TurretSubsystem(hardwareMap);
        TurretConfig.ENABLED = true;

        telemetry.addLine("=== Turret Goal Tracker Test Ready ===");
        telemetry.addLine("Drive with sticks (driveOrHold enabled).");
        telemetry.addLine("Turret tracks Goal 1 (Y < 72) and Goal 2 (Y >= 72).");
        telemetry.addLine("Use Bumpers to trim aim +/- 1° live.");
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
            // 2. Read Robot Pose from Pedro
            // -----------------------------------------------------------
            Pose currentPose = follower.pose();
            double robotX = currentPose.x();
            double robotY = currentPose.y();
            // Pedro heading is in radians; convert to degrees
            double robotHeadingDeg = Math.toDegrees(currentPose.heading());

            // -----------------------------------------------------------
            // 3. Goal Selection (Split at Y = 72.0)
            // -----------------------------------------------------------
            boolean useUpperGoal = robotY >= Y_SWITCH_LINE;
            double targetGoalX = useUpperGoal ? GOAL_2_X : GOAL_1_X;
            double targetGoalY = useUpperGoal ? GOAL_2_Y : GOAL_1_Y;

            // -----------------------------------------------------------
            // 4. Target Angle Math
            // -----------------------------------------------------------
            double dx = targetGoalX - robotX;
            double dy = targetGoalY - robotY;
            double distanceToGoal = Math.hypot(dx, dy);

            // Angle from robot to goal on field
            double fieldAngleToGoal = Math.toDegrees(Math.atan2(dy, dx));

            // Relative angle from robot heading
            double relativeAngle = fieldAngleToGoal - robotHeadingDeg;

            // Normalize to [-180°, +180°]
            while (relativeAngle > 180.0) relativeAngle -= 360.0;
            while (relativeAngle < -180.0) relativeAngle += 360.0;

            // Turret sign convention: Right = Positive (+), Left = Negative (-)
            // Pedro CCW: Left = Positive (+), Right = Negative (-)
            // Invert relative angle so Left turns negative, Right turns positive
            double turretTargetAngle = -relativeAngle + manualTrimDeg;

            // -----------------------------------------------------------
            // 5. Driver Aim Trim (Bumpers)
            // -----------------------------------------------------------
            if (gamepad1.right_bumper && !lastRightBumper) manualTrimDeg += 1.0;
            if (gamepad1.left_bumper && !lastLeftBumper)   manualTrimDeg -= 1.0;
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;

            // -----------------------------------------------------------
            // 6. Update Turret
            // -----------------------------------------------------------
            turret.setTargetAngle(turretTargetAngle);
            turret.update();

            // -----------------------------------------------------------
            // 7. Telemetry & Graphing
            // -----------------------------------------------------------
            telemetry.addData("Active Goal", useUpperGoal ? "GOAL 2 (Y >= 72)" : "GOAL 1 (Y < 72)");
            telemetry.addData("Goal Coords", "(%.1f, %.1f)", targetGoalX, targetGoalY);
            telemetry.addData("Distance to Goal", "%.1f in", distanceToGoal);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Robot X", "%.2f", robotX);
            telemetry.addData("Robot Y", "%.2f", robotY);
            telemetry.addData("Robot Heading", "%.1f°", robotHeadingDeg);

            telemetry.addLine("--------------------------------");
            telemetry.addData("Turret Target", "%.1f°", turret.getTargetAngle());
            telemetry.addData("Turret Actual", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("Turret Error", "%.1f°", turret.getTargetAngle() - turret.getCurrentAngle());
            telemetry.addData("Manual Trim", "%.1f°", manualTrimDeg);

            telemetry.update();
        }

        turret.stop();
    }
}