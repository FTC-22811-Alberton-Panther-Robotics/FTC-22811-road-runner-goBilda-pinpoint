/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Mr. Morris: This OpMode is based on the RobotAutoDriveByTime_Linear sample code.
 * I have adapted it to utilize the RobotHardware class.
 * I also added a configuration menu during robot initialization and sections for various
 * versions of the code dependent on alliance and starting location.
 *
 * At some point I hope to move to RoadRunner or at least an Encoder based auto, but for now
 * time based will get us started.
 */

/* The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 */

@Autonomous(name="Auto Drive By Time", group="Auto", preselectTeleOp = "TeleopAlternateButtonConfiguration")
//@Disabled
public class CompetitionAutoDriveByTime extends LinearOpMode {

    // Create a org.firstinspires.ftc.teamcode.RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this, telemetry);

    /* Declare OpMode members. */
    int legNumber = 0;
    boolean isRedAlliance = true;
    boolean isLeftStart = true;
    double startingPause = 0;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Select Alliance, starting point, and route options
        configAutonomous();

        // Wait for the game to start (driver presses START)
        waitForStart();

        ElapsedTime runTime = new ElapsedTime();
        while(runTime.seconds() < startingPause && opModeIsActive()); // do nothing during starting pause period
        robot.updateArmState();
        // Red Alliance, Right start (closer to observation zone / human player)
        if (isRedAlliance && !isLeftStart) {
            // Strafe right for 2.2 second
            driveForTime(2.2, .01, 0, .5);
            // Drive left
            driveForTime(.88, 0, 0, -.6);
            //drives forward a bit
            driveForTime(1.7, .5, 0, 0);
            // turns clockwise
            driveForTime(.67, 0, 0.5, 0);
            //drives forward a bit more

            /// First Sample
            // drives back to the right
            driveForTime(2, 0, 0, 0.5);
            //drives back to left
            driveForTime(2, 0, 0, -0.5);

            /// Second Sample
            //drives forward a little bit more
            driveForTime(.27, .5, 0, 0);
            // rotates clockwise
            driveForTime(.07, 0, 0.5, 0);
            // drives to right
            driveForTime(1.8, 0, 0, 0.5);
            //drives back left
            driveForTime(2, 0, 0, -0.5);

            /// Third Sample
            // drive forward a little
            driveForTime(.3, .5, 0, 0);
            // turns clockwise a bit more
            driveForTime(.1, 0, 0.5, 0);
            // drive  right
            driveForTime(1.9, 0, 0, 0.5);
            //drives left
            driveForTime(1.8, 0, 0, -0.5);

            ///drives to rung
            driveForTime(1,-.5,0,0);

            // Stop
            driveForTime(1, 0, 0, 0);
            robot.setWristPosition(.2);
            robot.setArmAngle(50);
           while (robot.getArmState() != RobotHardware.ArmState.HOLDING_POSITION && opModeIsActive()){
                robot.updateArmState();
            }
        }

        // Blue Alliance, Right start (closer to observation zone / human player)
        if (!isRedAlliance && !isLeftStart) {
           driveForTime(.1, .01, 0, 0);
            // Strafe right for 2.2 second
            driveForTime(2.2, .01, 0, .5);
            // Drive left
            driveForTime(.88, 0, 0, -.6);
            //drives forward a bit
            driveForTime(1.7, .5, 0, 0);
            // turns clockwise
            driveForTime(.67, 0, 0.5, 0);
            //drives forward a bit more

            /// First Sample
            // drives back to the right
            driveForTime(2, 0, 0, 0.5);
            //drives back to left
            driveForTime(2, 0, 0, -0.5);

            /// Second Sample
            //drives forward a little bit more
            driveForTime(.4, .5, 0, 0);
            // rotates clockwise
            driveForTime(.07, 0, 0.5, 0);
            // drives to right
            driveForTime(1.8, 0, 0, 0.5);
            //pause?
            driveForTime(0, 0, 0, 0);
            //drives back left
            driveForTime(2, 0, 0, -0.5);

            /// Third Sample
            // drive forward a little
            driveForTime(.4, .5, 0, 0);
//            // turns clockwise a bit more
            driveForTime(.1, 0, 0.5, 0);
//            // drive  right
            driveForTime(1.9, 0, 0, 0.5);
//            //drives left
            driveForTime(1.8, 0, 0, -0.5);

            ///drives to rung
            //turns 180
            driveForTime(.9, 0, .9, 0);

            driveForTime(1, .5, 0, 0);


            // Stop
            driveForTime(1, 0, 0, 0);
        }

        // Red Alliance, Left start (closer to net zone / baskets)
        if (isRedAlliance && isLeftStart) {
            // 1: Strafe left for 2.2 second
            driveForTime(2.2, .01, 0, -.5);
            // 2: Drive right
            driveForTime(.88, 0, 0, .6);
            // 3: drives forward a bit
            driveForTime(1.7, .5, 0, 0);
            // 4: turns counterclockwise
            driveForTime(.65,0,-0.5,0);

            /// First Sample
            // 5: Drive forward
            driveForTime(.25, .5, 0, 0);
            // 6: drives back to the left
            driveForTime(2, 0, 0, -0.5);
            // 7: drives back up
            driveForTime(2, 0, 0, 0.5);

            /// Second Sample
            // 8: drives forward a little bit more
            driveForTime(.27, .5, 0, 0);
            // 9: rotates a bit
            driveForTime(.1, 0, -0.5, 0);
            // 10: drives to left
            driveForTime(1.8, 0, 0, -0.5);
            // 11: drives back right
            driveForTime(2.1, 0, 0, 0.5);

            /// Third Sample
            // 12: drive forward a little
            driveForTime(.3, .5, 0, 0);
            // 13: turns a bit more
            driveForTime(.1, 0, -0.5, 0);
            // 14: drive  down
            driveForTime(1.9, 0, 0, -0.5);
            // 15: drives back up
            driveForTime(1.9, 0, 0, 0.5);

            ///drives to rung
            // 16:
            driveForTime(1, -.5, 0, 0);

            // Stop
            driveForTime(1, 0, 0, 0);
        }

        // Blue Alliance, Left start (closer to net zone / baskets)
        if (!isRedAlliance && isLeftStart) {
            // Strafe left for 2.2 second
            driveForTime(2.2, .01, 0, -.5);
            // Drive right
            driveForTime(.9, 0, 0, .6);
            //drives forward a bit
            driveForTime(1.7, .5, 0, 0);
            // turns counterclockwise
            driveForTime(.65, 0, -0.5, 0);
            //drives forward a bit more

            /// First Sample
            driveForTime(.2, .5, 0, 0);
            // drives back to the left
            driveForTime(2, 0, 0, -0.5);
            //drives back up
            driveForTime(1.55, 0, 0, 0.5);

            /// Second Sample
            //drives forward a little bit more
            driveForTime(.28, .5, 0, 0);
            // rotates a bit
            driveForTime(.07, 0, -0.5, 0);
            // drives to left
            driveForTime(1.8, 0, 0, -0.5);
            //drives back right
            driveForTime(2.1, 0, 0, 0.5);

            /// Third Sample
            // drive forward a little
            driveForTime(.3, .5, 0, 0);
            // turns a bit more
            driveForTime(.1, 0, -0.5, 0);
            // drive  down
            driveForTime(1.9, 0, 0, -0.5);
            //drives back up
            driveForTime(1.7, 0, 0, 0.5);

            ///drives to rung
            driveForTime(1, -.5, 0, 0);

            // Stop
            driveForTime(1, 0, 0, 0);

        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    /**
     * @param time       Time in seconds to complete the action.
     * @param forward     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param turn       Turn speed (-1.0 to 1.0) +ve is right
     * @param strafe     Right/Left strafing (-1.0 to 1.0) +ve is right
     * This function is used for just turning.
     */
    private void driveForTime(double time, double forward, double turn, double strafe) {
        legNumber += 1;
        robot.mecanumDrive(forward, strafe, turn);
        ElapsedTime legTime = new ElapsedTime();
        while (opModeIsActive() && (legTime.seconds() < time)) {
            telemetry.addData("Path", "Leg ", legNumber, ": %4.1f S Elapsed", legTime.seconds());
            telemetry.update();
        }
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
        while(!isStarted() && !configComplete){
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
}