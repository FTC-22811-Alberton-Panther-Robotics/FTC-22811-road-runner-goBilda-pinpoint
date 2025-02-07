//package org.firstinspires.ftc.teamcode.in_development;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//
//@Autonomous(name="RR Evan Auto Action Testing")
//@Disabled
//public class RREvanActionTest extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
//        Servo intakeLeft = hardwareMap.servo.get("servo");
//
//        waitForStart();
//
//        Actions.runBlocking(
//            drive.actionBuilder(new Pose2d(0,0,0))
//               // .lineToX(64)
//               // .stopAndAdd(new PatientServoAction(intakeLeft, 1))
//               // .lineToX(0)
//                .stopAndAdd(new ServoAction(intakeLeft, 0.5)) // Move to middle
//                .stopAndAdd(new ServoAction(intakeLeft, 0.0)) // Move to minimum
//                .stopAndAdd(new ServoAction(intakeLeft, 1.0)) // Move to maximum
//                .build());
//
//    }
//
//   // public class MotorAction implements Action {
//     //   DcMotor motor;
//
//       // public MotorAction(Servo s, double p) {
//           // this.motor = m;
//         //   this. = p;
//       // }
//
//        //@Override
//        //public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//          //  motor.setPosition(position);
//          // return false;
//        //}
//
//   // }
//    public static class ServoAction implements Action {
//        Servo servo;
//        double position;
//
//        public ServoAction(Servo s, double p) {
//            this.servo = s;
//            this.position = p;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            servo.setPosition(position);
//            return false;
//        }
//    }
//
//    public class PatientServoAction implements Action {
//        Servo servo;
//        double position;
//        ElapsedTime timer;
//
//        public PatientServoAction(Servo s, double p) {
//            this.servo = s;
//            this.position = p;
//        }
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            if (timer == null){
//                timer = new ElapsedTime();
//                servo.setPosition(position);
//            }
//
//            // do we need to keep running?
//            return timer.seconds() < 3;
//        }
//    }
//}
