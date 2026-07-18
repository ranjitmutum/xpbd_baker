package xpbd.regression;

import constraints.DistanceConstraint;
import core.XPBDEngine;
import models.Particle;
import models.Vector3;
import xpbd.baker.BoneMapper;
import xpbd.baker.PhysicsBaker;
import xpbd.loader.BedrockModelData;

import java.util.List;

/** Dependency-light regression checks for the remaining simulation core. */
public final class PhysicsRegressionTests {
    private PhysicsRegressionTests() {
    }

    public static void main(String[] args) {
        fixedParticlesAreKinematic();
        invalidTimeStepsAreRejected();
        distanceConstraintMaintainsLength();
        windMovesDynamicParticles();
        modelResetClearsPerBoneState();
        bakerProducesFrames();
        System.out.println("PhysicsRegressionTests: all checks passed");
    }

    private static void fixedParticlesAreKinematic() {
        XPBDEngine engine = new XPBDEngine();
        engine.setGravity(new Vector3(0, 0, 0));
        Particle fixed = new Particle(0);
        fixed.setPosition(new Vector3(2, 3, 4));
        fixed.setVelocity(new Vector3(10, -5, 1));
        engine.addParticle(fixed);

        engine.step(0.25);

        assertVector(fixed.getPosition(), 2, 3, 4, 1e-12,
                "fixed particle moved");
    }

    private static void invalidTimeStepsAreRejected() {
        XPBDEngine engine = new XPBDEngine();
        expectIllegalArgument(() -> engine.step(0), "zero dt was accepted");
        expectIllegalArgument(() -> engine.step(Double.NaN), "NaN dt was accepted");
        expectIllegalArgument(() -> engine.step(Double.POSITIVE_INFINITY),
                "infinite dt was accepted");
    }

    private static void distanceConstraintMaintainsLength() {
        XPBDEngine engine = new XPBDEngine();
        engine.setGravity(new Vector3(0, 0, 0));
        engine.setSolverIterations(8);

        Particle fixed = new Particle(0);
        fixed.setPosition(new Vector3(0, 0, 0));
        Particle dynamic = new Particle(1);
        dynamic.setPosition(new Vector3(3, 0, 0));
        engine.addParticle(fixed);
        engine.addParticle(dynamic);
        engine.addConstraint(new DistanceConstraint(0, 1, 1, 0, 0));

        engine.step(1.0 / 60.0);

        double dx = dynamic.getPosition().x - fixed.getPosition().x;
        double dy = dynamic.getPosition().y - fixed.getPosition().y;
        double dz = dynamic.getPosition().z - fixed.getPosition().z;
        assertClose(Math.sqrt(dx * dx + dy * dy + dz * dz), 1, 1e-8,
                "distance constraint did not restore rest length");
    }

    private static void windMovesDynamicParticles() {
        XPBDEngine engine = new XPBDEngine();
        engine.setGravity(new Vector3(0, 0, 0));
        engine.setAerodynamics(new Vector3(5, 0, 0), 1, 0);
        Particle particle = new Particle(1);
        particle.setPosition(new Vector3(0, 0, 0));
        engine.addParticle(particle);

        engine.step(0.1);

        if (!(particle.getPosition().x > 0)) {
            throw new AssertionError("wind did not move the dynamic particle");
        }
        assertFinite(particle.getPosition(), "wind produced a non-finite position");
    }

    private static void modelResetClearsPerBoneState() {
        BoneMapper mapper = twoBoneMapper();
        BoneMapper.BonePhysicsConfig override = new BoneMapper.BonePhysicsConfig();
        override.particleMass = 2.0;
        mapper.setBoneConfig("child", override);
        mapper.buildParticleMapping();

        mapper.resetModelState();

        if (!mapper.getPhysicsBones().isEmpty()
                || mapper.getBoneConfig("child") != null
                || mapper.getParticleIndex("child") >= 0) {
            throw new AssertionError("model-specific physics state leaked across reset");
        }
    }

    private static void bakerProducesFrames() {
        BoneMapper mapper = twoBoneMapper();
        try (PhysicsBaker baker = new PhysicsBaker(mapper)) {
            baker.initialize();
            baker.runSteps(2);

            if (baker.getFrames().isEmpty()) {
                throw new AssertionError("baker produced no frames");
            }
            for (PhysicsBaker.BoneState state
                    : baker.getFrames().get(baker.getFrames().size() - 1).boneStates) {
                assertFinite(new Vector3(state.worldPosition[0], state.worldPosition[1],
                        state.worldPosition[2]), "baker produced a non-finite position");
            }
        }
    }

    private static BoneMapper twoBoneMapper() {
        BedrockModelData.Bone root = new BedrockModelData.Bone();
        root.name = "root";
        root.pivot = new double[]{0, 0, 0};

        BedrockModelData.Bone child = new BedrockModelData.Bone();
        child.name = "child";
        child.parent = "root";
        child.pivot = new double[]{0, -1, 0};

        BoneMapper mapper = new BoneMapper(List.of(root, child));
        mapper.addPhysicsBone(root.name);
        mapper.addPhysicsBone(child.name);
        return mapper;
    }

    private static void expectIllegalArgument(Runnable action, String message) {
        try {
            action.run();
        } catch (IllegalArgumentException expected) {
            return;
        }
        throw new AssertionError(message);
    }

    private static void assertVector(Vector3 actual, double x, double y, double z,
                                     double epsilon, String message) {
        assertClose(actual.x, x, epsilon, message + " (x)");
        assertClose(actual.y, y, epsilon, message + " (y)");
        assertClose(actual.z, z, epsilon, message + " (z)");
    }

    private static void assertFinite(Vector3 actual, String message) {
        if (!Double.isFinite(actual.x) || !Double.isFinite(actual.y)
                || !Double.isFinite(actual.z)) {
            throw new AssertionError(message + ": " + actual);
        }
    }

    private static void assertClose(double actual, double expected, double epsilon,
                                    String message) {
        if (!Double.isFinite(actual) || !Double.isFinite(expected)
                || Math.abs(actual - expected) > epsilon) {
            throw new AssertionError(message + ": expected " + expected
                    + ", actual " + actual);
        }
    }
}
