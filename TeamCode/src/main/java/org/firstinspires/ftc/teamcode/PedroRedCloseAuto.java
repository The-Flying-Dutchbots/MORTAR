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

@Autonomous(name = "PedroRedClose", group = "Autonomous")
@Configurable // Panels
public class PedroRedCloseAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

     ShooterSys shooterSys = new ShooterSys();
     IntakeSys intakeSys = new IntakeSys();
     VisionSys visionSys = new VisionSys();
    ElapsedTime utilityTimer = new ElapsedTime();


    private int aprilTagSeen;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.68102288021534, 122.68102288021534, Math.toRadians(225)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        shooterSys.init(hardwareMap);
        intakeSys.init(hardwareMap);
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

        public PathChain Shoot1;
        public PathChain Check;
        public PathChain LeftLineSetup;
        public PathChain LeftLineLoad;
        public PathChain LeftLineScore;
        public PathChain MiddleLineSetup;
        public PathChain MiddleLineLoad;
        public PathChain MiddleLineScore;
        public PathChain RightLineSetup;
        public PathChain RightLineLoad;
        public PathChain RightLineScore;

        public Paths(Follower follower) {
            Shoot1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.681, 122.681), new Pose(112.022, 111.634))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-135),
                            Math.toRadians(-135)
                    )
                    .build();

            Check = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(112.022, 111.634), new Pose(81.594, 109.114))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(-135),
                            Math.toRadians(-90)
                    )

                    .build();

            LeftLineSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(81.594, 109.114),
                                    new Pose(81.206, 86.051),
                                    new Pose(99.618, 83.338)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            LeftLineLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(99.618, 83.338), new Pose(129.658, 83.532))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            LeftLineScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.658, 83.532),
                                    new Pose(100.393, 76.748),
                                    new Pose(87.408, 86.439)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                    .build();

            MiddleLineSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.408, 86.439),
                                    new Pose(71.322, 62.794),
                                    new Pose(101.943, 59.499)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                    .build();

            MiddleLineLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.943, 59.499), new Pose(133.922, 58.143))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            MiddleLineScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(133.922, 58.143),
                                    new Pose(98.067, 74.616),
                                    new Pose(87.408, 86.439)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                    .build();

            RightLineSetup = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.408, 86.439),
                                    new Pose(62.988, 51.166),
                                    new Pose(100.781, 35.467)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-135), Math.toRadians(0))
                    .build();

            RightLineLoad = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.781, 35.467), new Pose(131.402, 35.273))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            RightLineScore = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.402, 35.273),
                                    new Pose(87.795, 63.569),
                                    new Pose(87.408, 86.245)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-135))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState){
            case 1: // move to preload shooting position
                shooterSys.startShooting();
                follower.followPath(paths.Shoot1);
                pathState++;
                break;

            case 2: // start intake to shoot preloads
                if(!follower.isBusy()){
                    utilityTimer.reset();
                    intakeSys.shootStart();
                    pathState++;
                }
                break;
            case 3: // wait to shoot balls then go to the check position
                if(utilityTimer.seconds() > 5.0){
                    follower.followPath(paths.Check);
                    intakeSys.intakeStop();
                    shooterSys.stopShooting();
                    pathState++;
                }
                break;
            case 4: // once at check, determine next path
                aprilTagSeen = visionSys.getTagID();
                if(!follower.isBusy()){
                   if(aprilTagSeen == org.firstinspires.ftc.teamcode.Constants.PPG_TAG_ID){
                       pathState = 5; // move to PPG line
                   }else if(aprilTagSeen == org.firstinspires.ftc.teamcode.Constants.PGP_TAG_ID){
                       pathState = 0000; // move to PGP line
                   } else if (aprilTagSeen == org.firstinspires.ftc.teamcode.Constants.GPP_TAG_ID) {
                       pathState = 0000; // move to GPP line
                   }else{
                       pathState++;
                   }
                }
                break;
            case 5: // does the left most line intaking setup and set hood position
                shooterSys.setHoodState(ShooterSys.HoodState.MID);
                follower.followPath(paths.LeftLineSetup);
                pathState++;
                break;
            case 6: //begin intaking the left line pieces
                if(!follower.isBusy()){
                    intakeSys.intakeStart();
                    follower.followPath(paths.LeftLineLoad);
                    pathState++;
                }
                break;
            case 7: // drive through the line and intake, then begin path to score
                intakeSys.intakeStart();
                if(!follower.isBusy()){
                    intakeSys.intakeStop();
                    follower.followPath(paths.LeftLineScore);
                    shooterSys.startShooting();
                    pathState++;
                }
                break;
            case 8: //get to shooter position and move to wait state
                if(!follower.isBusy()){
                    intakeSys.shootStart();

                    utilityTimer.reset();
                    pathState++;
                }
                break;
            case  9:
                if (utilityTimer.seconds() > 5.0){

                }

        }


        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}