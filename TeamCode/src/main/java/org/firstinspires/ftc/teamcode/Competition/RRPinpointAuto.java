package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;

@Autonomous(name = "RR Pinpoint Auto", group = "RoadRunner")


//@Disabled
public class RRPinpointAuto extends OpMode {


    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.


    int legNumber = 0;
    boolean isRedAlliance = true;
    boolean isLeftStart = true;
    double startingPause = 0;
    private PinpointDrive drive;
    private Pose2d initialPose;

    @Override
    public void init() {
        // instantiate the PinpointDrive at a particular pose.
        initialPose = new Pose2d(11.8, 61.7, Math.toRadians(-90));
        drive = new PinpointDrive(hardwareMap, initialPose);
    }

    @Override
    public void start(){
        Actions.runBlocking(drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(-5))
                .lineToX(50)
                .splineToLinearHeading(new Pose2d(36,30,Math.toRadians(-90)), Math.toRadians(-95))
               .splineToLinearHeading(new Pose2d(43,12,Math.toRadians(-15)), Math.toRadians(-25))
                .setTangent(Math.toRadians(75))
                .lineToY(56)
                .lineToY(30)
                .splineToLinearHeading(new Pose2d(50,12,Math.toRadians(-15)), Math.toRadians(-25))
                .setTangent(Math.toRadians(80))
                .lineToY(53)
                .lineToY(30)
                .splineToLinearHeading(new Pose2d(61,12,Math.toRadians(0)), Math.toRadians(-25))
                .setTangent(Math.toRadians(90))
                .lineToY(53)
                .splineToLinearHeading(new Pose2d(40,40,Math.toRadians(0)), Math.toRadians(-90))
                .lineToY(15)
//                .splineToLinearHeading(new Pose2d(25, 12, Math.toRadians(0)), Math.toRadians(0))
                .build());
    }

    @Override
    public void loop(){

    }
}
