package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.Alliance.Alliance;
import org.firstinspires.ftc.teamcode.configurables.AutoConfig;
import org.firstinspires.ftc.teamcode.configurables.FieldConfig;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.configurables.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public abstract class BaseAuto extends LinearOpMode {
    protected final Alliance alliance;

    protected Follower follower;
    protected ShooterSubsystem shooter;
    protected IntakeSubsystem intake;
    protected TurretSubsystem turret;

    private int pathState = 0;
    private final ElapsedTime firingTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();

    private boolean readyToFire = false;
    private boolean isFeeding = false;

    protected final PoseFactory poseFactory;

    // Coordinate Poses (Initialized in constructor with alliance mirroring)
    private final Pose start;
    private final Pose path1;
    private final Pose point2;
    private final Pose point3;
    private final Pose point3Control1;
    private final Pose point3Control2;
    private final Pose point3Control3;
    private final Pose point4Start;
    private final Pose point4;
    private final Pose point4Control1;
    private final Pose point5;
    private final Pose point6;
    private final Pose point6Control1;
    private final Pose point6Control2;
    private final Pose point7Start;
    private final Pose point7;
    private final Pose point7Control1;
    private final Pose point7Control2;
    private final Pose point7Control3;
    private final Pose point7Control4;
    private final Pose point8;

    public BaseAuto(Alliance alliance) {
        this.alliance = alliance;

        // Blue uses standard frame; Red mirrors across both centerlines (72, 72)
        if (alliance == Alliance.BLUE) {
            this.poseFactory = PoseFactory.degrees();
        } else {
            this.poseFactory = PoseFactory.degrees().mirrorX(72.0).mirrorY(72.0);
        }

        // Poses automatically inherit mirroring from poseFactory
        this.start = poseFactory.of(82.75, 133.75, 270);
        this.path1 = poseFactory.of(113.75, 94.75, 0);
        this.point2 = poseFactory.of(125.5, 94.75, 0);
        this.point3 = poseFactory.of(81.75, 8.75, 90);
        this.point3Control1 = poseFactory.of(79.75, 93.75, 0);
        this.point3Control2 = poseFactory.of(137.75, 17.75, 0);
        this.point3Control3 = poseFactory.of(81.75, 29.75, 0);
        this.point4Start = poseFactory.of(81.75, 8.75, 90);
        this.point4 = poseFactory.of(94.75, 28.75, 270);
        this.point4Control1 = poseFactory.of(86.75, 21.75, 0);
        this.point5 = poseFactory.of(94.75, 15, 270);
        this.point6 = poseFactory.of(81.75, 9.75, 90);
        this.point6Control1 = poseFactory.of(95.75, 31.75, 0);
        this.point6Control2 = poseFactory.of(81.75, 20.75, 0);
        this.point7Start = poseFactory.of(81.75, 9.75, 90);
        this.point7 = poseFactory.of(130.75, 79.75, 90);
        this.point7Control1 = poseFactory.of(88.75, 23.75, 0);
        this.point7Control2 = poseFactory.of(119.75, 21.75, 0);
        this.point7Control3 = poseFactory.of(105.75, 48.75, 0);
        this.point7Control4 = poseFactory.of(99.75, 86.75, 0);
        this.point8 = poseFactory.of(130.75, 47.75, 90);
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        TurretConfig.ENABLED = true;
        intake.setRaiseFlowerWhenFull(false); // Auto keeps the flower arm down
        shooter.spinUp();

        intake.flowerUp();
        while (!isStarted() && !isStopRequested()) {
            intake.update(false);
            telemetry.addData("Alliance", alliance);
            telemetry.addLine("Auto Initialized. Flower Arm UP.");
            telemetry.update();
        }

        Constants.foresightConfig.maxPathSpeed.set(AutoConfig.MAX_PATH_SPEED);
        intake.flowerDown();

        setPathState(0);

        while (opModeIsActive()) {
            follower.update();

            updateAimAndFlywheel();
            intake.update(isFeeding);
            shooter.update();
            turret.update();

            autonomousPathUpdate();

            telemetry.addData("Alliance", alliance);
            telemetry.addData("Path State", pathState);
            telemetry.addData("Robot Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                    follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.update();
        }

        PoseStorage.setPose(follower.pose());

        turret.stop();
        shooter.spinDown();
        intake.stop();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                handleFiring(1, path1());
                break;

            case 1:
                intake.intake();
                if (!follower.isBusy()) {
                    follower.follow(path2());
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    intakeTimer.reset();
                    setPathState(3);
                }
                break;

            case 3:
                if (intake.isConfirmedFull() || intakeTimer.seconds() > AutoConfig.INTAKE_TIMEOUT_1) {
                    intake.stop();
                    follower.follow(path3());
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                handleFiring(6, path4());
                break;

            case 6:
                intake.intake();
                if (!follower.isBusy()) {
                    follower.follow(path5());
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    intakeTimer.reset();
                    setPathState(8);
                }
                break;

            case 8:
                if (intake.isConfirmedFull() || intakeTimer.seconds() > AutoConfig.INTAKE_TIMEOUT_2) {
                    intake.stop();
                    follower.follow(path6());
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                handleFiring(11, path7());
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.follow(path8());
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    setPathState(999);
                }
                break;

            case 999:
                PoseStorage.setPose(follower.pose());
                turret.stop();
                shooter.spinDown();
                intake.stop();
                break;
        }
    }

    private void handleFiring(int nextState, Path nextPath) {
        if (!follower.isBusy() && !readyToFire) {
            firingTimer.reset();
            readyToFire = true;
            isFeeding = false;
        }

        if (readyToFire) {
            double currentError = Math.abs(shooter.getTargetVelocity() - shooter.getCurrentVelocity());
            boolean isAtSpeed = currentError <= AutoConfig.RPM_TOLERANCE;
            boolean settled = firingTimer.seconds() >= AutoConfig.DELAY_BEFORE_FIRE;
            boolean timedOut = firingTimer.seconds() >= (AutoConfig.DELAY_BEFORE_FIRE + AutoConfig.MAX_SPINUP_WAIT);

            if (!isFeeding && (settled && isAtSpeed || timedOut)) {
                isFeeding = true;
                feedTimer.reset();
                shooter.openGate();
                intake.intakeShooting();
            }

            if (isFeeding) {
                shooter.openGate();
                intake.intakeShooting();

                if (feedTimer.seconds() >= AutoConfig.SHOOT_TIME) {
                    shooter.closeGate();
                    intake.stop();
                    readyToFire = false;
                    isFeeding = false;
                    if (nextPath != null) {
                        follower.follow(nextPath);
                    }
                    setPathState(nextState);
                }
            } else {
                shooter.closeGate();
                intake.stop();
            }
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
    }

    private void updateAimAndFlywheel() {
        Pose currentPose = follower.pose();
        double rx = currentPose.x();
        double ry = currentPose.y();
        double rh = Math.toDegrees(currentPose.heading());

        // Goal 1 = lower goal, Goal 2 = upper goal (mirrored automatically for Red)
        double g1X = FieldConfig.goalX(alliance, false);
        double g1Y = FieldConfig.goalY(alliance, false);
        double g2X = FieldConfig.goalX(alliance, true);
        double g2Y = FieldConfig.goalY(alliance, true);

        // Always aim at the NEAREST goal to where the robot is shooting
        double d1 = Math.hypot(g1X - rx, g1Y - ry);
        double d2 = Math.hypot(g2X - rx, g2Y - ry);

        boolean isGoal1Closer = d1 <= d2;
        double gx = isGoal1Closer ? g1X : g2X;
        double gy = isGoal1Closer ? g1Y : g2Y;
        double dist = isGoal1Closer ? d1 : d2;

        // Each alliance + goal side has its own trims (see AutoConfig)
        double turretTrim = AutoConfig.turretTrim(alliance, isGoal1Closer);
        double rpmTrim = AutoConfig.rpmTrim(alliance, isGoal1Closer);

        turret.aimAt(rx, ry, rh, gx, gy, turretTrim);
        shooter.setTargetVelocity(ShooterSubsystem.calculateVelocityFromDistance(dist) + rpmTrim);
    }

    private Path path1() { return Paths.line(start, path1).linear(start, path1); }
    private Path path2() { return Paths.line(path1, point2).tangent(); }
    private Path path3() { return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3).reverseTangent(); }
    private Path path4() { return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4); }
    private Path path5() { return Paths.line(point4, point5).constant(point5); }
    private Path path6() { return Paths.curve(point5, point6Control1, point6Control2, point6).reverseTangent(); }
    private Path path7() { return Paths.curve(point7Start, point7Control1, point7Control2, point7Control3, point7Control4, point7).linear(point7Start, point7); }
    private Path path8() { return Paths.line(point7, point8).reverseTangent(); }
}