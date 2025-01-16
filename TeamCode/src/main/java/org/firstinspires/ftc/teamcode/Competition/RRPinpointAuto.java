package org.firstinspires.ftc.teamcode.Competition;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.PinpointDrive;

@Autonomous(name = "RR Pinpoint Auto", group = "RoadRunner")


//@Disabled
public class RRPinpointAuto extends OpMode {

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

    /**
     * This method used a technique called latching with all the button presses where it only toggles
     * when you let go of the button. This keeps from accidentally registering multiple button presses
     * when you hold the button for too long.
     */
    private void configAutonomous(){
        int selection = 0;
        final int SELECTION_COUNT = 4; // Number of options in selection list.
        boolean dpadDownPressed = false, dpadUpPressed = false, dpadRightOrLeftPressed = false, dpadRightPressed = false, dpadLeftPressed = false;
        boolean configComplete = false;

        // This loops until the x
        while(!start() && !configComplete){
            // This is the first example of latching. This while loop can cycle hundreds of times a
            // second. This waits until the dpad_down button is not pressed before it increments
            // the selection
            if(gamepad1.dpad_down) dpadDownPressed = true;
            else if(dpadDownPressed && !gamepad1.dpad_down){
                dpadDownPressed = false;
                selection += 1;
                selection = selection % SELECTION_COUNT; // cycles around to beginning of list after the end
                // % means the remainder after dividing
            }

            if(gamepad1.dpad_up) dpadUpPressed = true;
            else if(dpadUpPressed && !gamepad1.dpad_up){
                dpadUpPressed = false;
                if (selection >0) selection -= 1;
            }

            // The following blocks display an arrow next to the option currently being selected and wait for
            // you to toggle that option by pressing either dpad_left or dpad_right
            telemetry.addLine("Press Dpad Up/Down to choose an option, and Left/Right to change options");
            if(selection == 1) {
                telemetry.addData("-->Alliance: ", isRedAlliance ?"Red": "Blue"); //This syntax is an inline if statement that can be used in simple cases
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isRedAlliance = !isRedAlliance;
                }
            } else telemetry.addData("Alliance: ", isRedAlliance ?"Red": "Blue");

            if(selection == 2) {
                telemetry.addData("-->Starting Position: ", isLeftStart ?"Left": "Right");
                if(gamepad1.dpad_right || gamepad1.dpad_left) dpadRightOrLeftPressed = true;
                else if(dpadRightOrLeftPressed && !(gamepad1.dpad_right || gamepad1.dpad_left)){
                    dpadRightOrLeftPressed = false;
                    isLeftStart = !isLeftStart;
                }
            } else telemetry.addData("Starting Position: ", isLeftStart ?"Left": "Right");

            if(selection == 3) {
                telemetry.addData("-->Starting Pause: ", startingPause +" seconds");
                if(gamepad1.dpad_right) dpadRightPressed = true;
                else if(dpadRightPressed && !gamepad1.dpad_right){
                    dpadRightPressed = false;
                    startingPause += 0.5;
                }
                if(gamepad1.dpad_left) dpadLeftPressed = true;
                else if(dpadLeftPressed && !gamepad1.dpad_left){
                    dpadLeftPressed = false;
                    startingPause -= 0.5;
                }
            } else telemetry.addData("Starting Pause: ", startingPause +" seconds");

            telemetry.addLine("\nPress [x] to complete");

            // Press x to end Autonomous Configuration
            if(gamepad1.x) configComplete = true;
            telemetry.update();
        }

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Autonomous Configuration Complete.  Press Play to start OpMode.");
        telemetry.update();
    }

    @Override
    public void loop() {

    }
}
