package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoPaths {
    public enum Alliance {
        RED,
        BLUE,
    };

    public Pose2d getBasketAutoStartPose(Alliance alliance) {
        switch (alliance) {
            case RED:
                return new Pose2d(0, 64, 0);
            case BLUE:
                return new Pose2d(0, -64, 0);
        }

        return new Pose2d(0, 0, 0);
    }

    public void getTrajectoryToBasket(Alliance alliance) {
        Pose2d startPose = getBasketAutoStartPose(alliance);

    }

}
