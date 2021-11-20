package org.frcteam2910.c2020.util;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.io.PathReader;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

public class AutonomousTrajectories {

    private static final double SAMPLE_DISTANCE = 0.1;

    private Trajectory fiveFootForward;
    private Trajectory fiveFootLeftSlow;
    private Trajectory fiveFootDiagonal;
    private Trajectory fiveFootDiagonalRotate;
    private Trajectory tenFootArc;
    private Trajectory complexTrajectory;

    public AutonomousTrajectories(TrajectoryConstraint[] trajectoryConstraints) throws IOException {
        TrajectoryConstraint[] slowConstraints = Arrays.copyOf(trajectoryConstraints, trajectoryConstraints.length + 1);
        slowConstraints[slowConstraints.length - 1] = new MaxVelocityConstraint(6.0 * 12.0);
        slowConstraints[slowConstraints.length - 2] = new MaxAccelerationConstraint(4.0 * 12.0);

        //assuming units are inches
        fiveFootForward = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        //not sure if this left, uses slow constraints
        fiveFootLeftSlow = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(0, 60))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        //straight line diagonal probably?
        fiveFootDiagonal = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 60))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

        //same as above but ends facing left? maybe?
        fiveFootDiagonalRotate = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(60, 60), Rotation2.fromDegrees(90.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );
        
        //not actually sure how arcs work
        tenFootArc = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .arcTo(new Vector2(60,60), new Vector2(90, 120))
                        .build(),
                slowConstraints, SAMPLE_DISTANCE
        );

        //
        complexTrajectory = new Trajectory(
                new SimplePathBuilder(new Vector2(0, 0), Rotation2.ZERO)
                        .lineTo(new Vector2(48, 48), Rotation2.fromDegrees(90.0))
                        .arcTo(new Vector2(0, 48), new Vector2(0, 0), Rotation2.fromDegrees(180.0))
                        .lineTo(new Vector2(48, 0), Rotation2.fromDegrees(270.0))
                        .build(),
                trajectoryConstraints, SAMPLE_DISTANCE
        );

    }

    private Path getPath(String name) throws IOException {
        InputStream in = getClass().getClassLoader().getResourceAsStream(name);
        if (in == null) {
            throw new FileNotFoundException("Path file not found: " + name);
        }

        try (PathReader reader = new PathReader(new InputStreamReader(in))) {
            return reader.read();
        }
    }

    public Trajectory getFiveFootForward() {
        return fiveFootForward;
    }

    public Trajectory getFiveFootLeftSlow() {
        return fiveFootLeftSlow;
    }

    public Trajectory getFiveFootDiagonal() {
        return fiveFootDiagonal;
    }

    public Trajectory getFiveFootDiagonalRotate() {
        return fiveFootDiagonalRotate;
    }

    public Trajectory getTenFootArc() {
        return tenFootArc;
    }

    public Trajectory getComplexTrajectory() {
        return complexTrajectory;
    }
}
