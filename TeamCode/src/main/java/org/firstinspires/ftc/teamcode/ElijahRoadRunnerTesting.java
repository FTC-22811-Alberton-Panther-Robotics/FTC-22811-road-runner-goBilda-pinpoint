package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "Elijah RoadRunner Testing")
public class ElijahRoadRunnerTesting  extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Servo servo= hardwareMap.servo.get("servo");

        waitForStart();
/// moves the robot
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .lineToX(30)
                        .stopAndAdd(new patientServoAction(servo, .5)))
                        .build();




    }

    ///makes the servo be able to be used
    public class ServoAction implements Action {
        Servo Servo;
        double position;
        public ServoAction(Servo s, double p) {
            this.Servo = s;
            this.position = p;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Servo.setPosition(position);
            return false;
        }
    }
    public class patientServoAction implements Action {
        Servo Servo;
        double position;
        ElapsedTime timer;

        boolean hasInitialized;
        public patientServoAction(Servo s, double p) {
            this.Servo = s;
            this.position = p;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!hasInitialized){
                timer = new ElapsedTime();
                Servo.setPosition(position);

            }


            // do we need to keep runing
            return timer.seconds() < 3;

        }
    }
}

