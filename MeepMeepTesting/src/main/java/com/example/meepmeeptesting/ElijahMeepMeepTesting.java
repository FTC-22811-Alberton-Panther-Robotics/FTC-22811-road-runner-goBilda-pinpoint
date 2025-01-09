package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ElijahMeepMeepTesting {
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
                .build());

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14.79164339476791)
                .build();

        bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(-11.8, 61.7, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-70))
                .lineToY(38)
                .waitSeconds(0)
//                .splineToLinearHeading(new Pose2d(,,Math.toRadians()), Math.toRadians())
//                .lineToY()

                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(-35, Math.toRadians(-140))
                .lineToX(-65)

                .build());



                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot1)
                .addEntity(bot2)
                .start();
    }
}