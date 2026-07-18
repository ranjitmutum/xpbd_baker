package xpbd.regression;

import constraints.VertexFaceCollisionConstraint;
import models.Particle;
import models.Vector3;
import org.junit.jupiter.api.Test;
import xpbd.baker.BodyColliderCache;
import xpbd.baker.BonePoseCalculator;
import xpbd.loader.BedrockModelData;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertTrue;

/** Repeatable typical-scale smoke benchmark; not a machine-independent score. */
final class CollisionPerformanceSmokeTest {
    private static final int PARTICLE_COUNT = 128;
    private static final int CUBE_COUNT = 32;
    private static final int ITERATIONS = 8;
    private static final int WARMUP_SAMPLES = 40;
    private static final int MEASURED_SAMPLES = 80;

    @Test
    void typicalScaleMedianAndP95StayWithinSmokeBudget() {
        BedrockModelData.Bone body = new BedrockModelData.Bone();
        body.name = "body";
        for (int i = 0; i < CUBE_COUNT; i++) {
            BedrockModelData.Cube cube = new BedrockModelData.Cube();
            cube.origin = new double[]{(i % 8) * 3.0, (i / 8) * 3.0, -1};
            cube.size = new double[]{2, 2, 2};
            body.cubes.add(cube);
        }
        List<BedrockModelData.Bone> bones = List.of(body);
        BodyColliderCache cache = new BodyColliderCache(bones, Set.of("body"));
        cache.initialize(BonePoseCalculator.calculate(bones, null, 0));
        cache.advance(BonePoseCalculator.calculate(bones, null, 0), true);

        Particle[] particles = new Particle[PARTICLE_COUNT];
        int[] indices = new int[PARTICLE_COUNT];
        for (int i = 0; i < particles.length; i++) {
            particles[i] = new Particle(1);
            indices[i] = i;
        }
        VertexFaceCollisionConstraint constraint = new VertexFaceCollisionConstraint(
                indices, particles.length, cache, 0.1);
        for (int i = 0; i < WARMUP_SAMPLES; i++) {
            runSample(particles, constraint);
        }

        long[] samples = new long[MEASURED_SAMPLES];
        for (int i = 0; i < samples.length; i++) {
            resetParticles(particles);
            constraint.resetLambda();
            long start = System.nanoTime();
            for (int iteration = 0; iteration < ITERATIONS; iteration++) {
                constraint.solve(particles, 1.0 / 60.0);
            }
            constraint.postSolveVelocity(particles);
            samples[i] = System.nanoTime() - start;
        }
        Arrays.sort(samples);
        double medianMs = samples[samples.length / 2] / 1_000_000.0;
        double p95Ms = samples[(int) Math.ceil(samples.length * 0.95) - 1]
                / 1_000_000.0;
        System.out.printf(Locale.US,
                "Collision benchmark P=%d C=%d I=%d: median %.3f ms, p95 %.3f ms%n",
                PARTICLE_COUNT, CUBE_COUNT, ITERATIONS, medianMs, p95Ms);

        assertTrue(p95Ms < 500,
                "typical collision smoke benchmark exceeded 500 ms p95: " + p95Ms);
    }

    private static void runSample(Particle[] particles,
                                  VertexFaceCollisionConstraint constraint) {
        resetParticles(particles);
        constraint.resetLambda();
        for (int iteration = 0; iteration < ITERATIONS; iteration++) {
            constraint.solve(particles, 1.0 / 60.0);
        }
        constraint.postSolveVelocity(particles);
    }

    private static void resetParticles(Particle[] particles) {
        for (int i = 0; i < particles.length; i++) {
            int cube = i % CUBE_COUNT;
            double x = (cube % 8) * 3.0 + 1;
            double y = (cube / 8) * 3.0 + 1;
            particles[i].setPosition(new Vector3(x, y, 0));
            particles[i].setPrevPosition(new Vector3(x, y, 0));
            particles[i].getVelocity().set(0, 0, 0);
        }
    }
}
