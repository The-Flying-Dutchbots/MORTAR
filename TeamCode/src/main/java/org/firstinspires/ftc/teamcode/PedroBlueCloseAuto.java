package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSys;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSys;
import org.firstinspires.ftc.teamcode.subsystems.VisionSys;



@Autonomous(name = "PedroBlueCloseAuto", group = "Autonomous")
@Configurable // Panels
public class PedroBlueCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    ShooterSys shooterSys = new ShooterSys();
    IntakeSys intakeSys = new IntakeSys();
    VisionSys visionSys = new VisionSys();
    ElapsedTime utilityTimer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(18.4, 120.9, Math.toRadians(-37)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intakeSys.init(hardwareMap);
        shooterSys.init(hardwareMap);
        visionSys.init(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain ScorePreloadsClose;
        public PathChain IntakeLeftSetup;
        public PathChain IntakeLeftLoadOne;
        public PathChain IntakeLeftLoadTwo;
        public PathChain LeftToScore;
        public PathChain IntakeMiddleSetup;
        public PathChain IntakeMiddleLoadOne;
        public PathChain IntakeMiddleLoadTwo;
        public PathChain MiddleToScore;
        public PathChain LEAVE;

        public Paths(Follower follower) {
            ScorePreloadsClose = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.412, 120.937), new Pose(26.746, 116.285))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))
                    .build();

            IntakeLeftSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(26.746, 116.285),
                                    new Pose(47.483, 110.859),
                                    new Pose(43.026, 83.919)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();

            IntakeLeftLoadOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.026, 83.919), new Pose(28.684, 83.532))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeLeftLoadTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.684, 83.532), new Pose(16.086, 83.532))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            LeftToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16.086, 83.532), new Pose(55.623, 87.602))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            IntakeMiddleSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(55.623, 87.602),
                                    new Pose(57.755, 61.244),
                                    new Pose(41.669, 60.468)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();

            IntakeMiddleLoadOne = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(41.669, 60.468), new Pose(28.296, 59.887))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            IntakeMiddleLoadTwo = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(28.296, 59.887), new Pose(13.954, 58.724))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            MiddleToScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.954, 58.724),
                                    new Pose(52.522, 59.499),
                                    new Pose(55.623, 87.214)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))
                    .build();

            LEAVE = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55.623, 87.214), new Pose(39.149, 35.661))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1: // move to preload shooting position
                shooterSys.startShooting();
                follower.followPath(paths.ScorePreloadsClose);
                pathState++;
                break;

            case 2: // start intake to shoot preloads
                if (!follower.isBusy()) {
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 3: // wait to shoot balls then go to the left intake position
                shooterSys.startShooting();
                intakeSys.shootStart();
                if (utilityTimer.seconds() > 5.0) {
                    follower.followPath(paths.IntakeLeftSetup);
                    shooterSys.setHoodState(ShooterSys.HoodState.MID);
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 4: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeLeftLoadOne);
                    pathState++;
                }
                break;
            case 5:
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeLeftLoadTwo);
                    pathState++;
                }
                break;
            case 6:
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    intakeSys.intakeStop();
                    follower.followPath(paths.LeftToScore);
                    pathState++;
                }
                break;
            case 7:
                shooterSys.startShooting();
                if(!follower.isBusy()){
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 8:
                shooterSys.startShooting();
                if(utilityTimer.seconds() > 5.0){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    follower.followPath(paths.IntakeMiddleSetup);
                    pathState++;
                }
                break;
            case 9: // begin intake
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeMiddleLoadOne);
                    pathState++;
                }
                break;
            case 10:
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    follower.followPath(paths.IntakeMiddleLoadTwo);
                    pathState++;
                }
                break;
            case 11:
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    intakeSys.intakeStop();
                    shooterSys.startShooting();
                    follower.followPath(paths.MiddleToScore);
                    pathState++;
                }
                break;
            case 12:
                shooterSys.startShooting();
                if(!follower.isBusy()){
                    utilityTimer.reset();
                    pathState++;
                }
                break;
            case 13:
                intakeSys.shootStart();
                if(utilityTimer.seconds() >5.0){
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 14:
                follower.followPath(paths.LEAVE);
                break;

        }
        shooterSys.updateHood();

        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}

