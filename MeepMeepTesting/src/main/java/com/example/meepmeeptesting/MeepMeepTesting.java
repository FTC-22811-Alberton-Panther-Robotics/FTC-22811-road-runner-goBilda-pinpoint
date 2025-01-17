package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot1 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.79164339476791)
                .build();

        bot1.runAction(bot1.getDrive().actionBuilder(new Pose2d(37, 61.7, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .lineToX(49)
                .splineToLinearHeading(new Pose2d(37,35,Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(17)
                .splineToLinearHeading(new Pose2d(40,12,Math.toRadians(160)), Math.toRadians(75))
                .setTangent(Math.toRadians(75))
                .lineToY(58)
                .lineToY(12)
                .splineToLinearHeading(new Pose2d(53,12,Math.toRadians(180)), Math.toRadians(85))
                .setTangent(Math.toRadians(85))
                .lineToY(55)
                .lineToY(12)
                .splineToLinearHeading(new Pose2d(61,10,Math.toRadians(180)), Math.toRadians(90))
                .lineToY(46)
                .splineToLinearHeading(new Pose2d(25, 12, Math.toRadians(180)), Math.toRadians(160))
                .build());

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.79164339476791)
                .build();

        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(-11.8, 61.7, Math.toRadians(-90)))
                .setTangent(Math.toRadians(0))
                .lineToX(-49)
                .splineToLinearHeading(new Pose2d(-35,32,Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(20)
                .splineToLinearHeading(new Pose2d(-45,12,Math.toRadians(0)), Math.toRadians(90))
                .lineToY(53)
                .lineToY(12)
                .splineToLinearHeading(new Pose2d(-53,10,Math.toRadians(0)), Math.toRadians(90))
                .lineToY(53)
                .lineToY(12)
                .splineToLinearHeading(new Pose2d(-61,10,Math.toRadians(0)), Math.toRadians(90))
                .lineToY(50)
                //.splineToLinearHeading(new Pose2d(-25, 12, Math.toRadians(0)), Math.toRadians(20))
                .build());


                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}