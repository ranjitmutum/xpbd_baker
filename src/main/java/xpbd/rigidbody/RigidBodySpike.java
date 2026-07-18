package xpbd.rigidbody;

import xpbd.baker.BonePoseCalculator;
import xpbd.loader.BedrockModelData;
import xpbd.loader.ModelLoader;

import java.util.List;
import java.util.Map;
import java.util.ArrayList;
import java.util.logging.Logger;

/** 不启动 JavaFX、也不修改导出内容的命令行决策验证工具。 */
public final class RigidBodySpike {
    private static final double FIXED_DT = 1.0 / 120.0;
    private static final Logger LOGGER = Logger.getLogger(RigidBodySpike.class.getName());

    private RigidBodySpike() {
    }

    public static void main(String[] args) throws Exception {
        ModelReport model = args.length == 0 ? null : compileModel(args[0]);
        SpikeReport spike = runSyntheticKick();
        if (!spike.sweepDetected() || spike.maximumRibbonX() <= 0.05
                || spike.maximumPenetration() >= 0.3) {
            throw new IllegalStateException("rigid-body spike failed: " + spike);
        }
        LOGGER.info("Rigid-body spike passed: " + spike);
        if (model != null) LOGGER.info("Bedrock model compiled: " + model);
    }

    public static ModelReport compileModel(String modelPath) throws Exception {
        BedrockModelData.Geometry geometry = ModelLoader.load(modelPath);
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(geometry.bones, null, 0);
        int bodies = 0;
        int boxes = 0;
        int skipped = 0;
        List<RigidBodyBackend.BodyDefinition> definitions = new ArrayList<>();
        for (BedrockModelData.Bone bone : geometry.bones) {
            BedrockRigidBodyCompiler.Compilation compiled =
                    BedrockRigidBodyCompiler.compile(bone, poses.get(bone.name),
                            RigidBodyBackend.MotionType.STATIC, 0, 1);
            if (compiled.body().isPresent()) {
                bodies++;
                RigidBodyBackend.BodyDefinition definition =
                        compiled.body().orElseThrow();
                boxes += definition.boxes().size();
                definitions.add(definition);
            }
            skipped += compiled.skippedDegenerateCubeCount();
        }
        long start = System.nanoTime();
        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            for (RigidBodyBackend.BodyDefinition definition : definitions) {
                backend.createBody(definition);
            }
            backend.step(FIXED_DT);
        }
        double instantiationMilliseconds = (System.nanoTime() - start) / 1_000_000.0;
        return new ModelReport(geometry.bones.size(), bodies, boxes, skipped,
                instantiationMilliseconds);
    }

    public static SpikeReport runSyntheticKick() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            RigidBodyBackend.BodyHandle anchor = backend.createBody(kinematicBody(
                    "anchor", transform(0, 1.6, 0), 0.08, 0.08, 0.08));
            RigidBodyBackend.BodyHandle top = backend.createBody(dynamicBody(
                    "ribbon-top", transform(0, 0.9, 0), 0.12, 0.35, 0.2));
            RigidBodyBackend.BodyHandle bottom = backend.createBody(dynamicBody(
                    "ribbon-bottom", transform(0, 0.2, 0), 0.12, 0.35, 0.2));
            RigidBodyBackend.BodyHandle leg = backend.createBody(kinematicBody(
                    "leg", transform(-1.2, 0.2, 0), 0.18, 0.5, 0.3));
            RigidBodyBackend.JointSettings joint = new RigidBodyBackend.JointSettings(
                    new double[]{-0.7, -0.7, -0.7},
                    new double[]{0.7, 0.7, 0.7}, 12, 0.8);
            backend.addSpringJoint(anchor, top, transform(0, 1.25, 0), joint);
            backend.addSpringJoint(top, bottom, transform(0, 0.55, 0), joint);

            boolean sweepDetected = false;
            double maximumRibbonX = Double.NEGATIVE_INFINITY;
            double maximumPenetration = 0;
            RigidBodyBackend.Transform previousLeg = transform(-1.2, 0.2, 0);
            for (int step = 0; step < 12; step++) {
                double x = -1.2 + 2.4 * (step + 1) / 12.0;
                RigidBodyBackend.Transform nextLeg = transform(x, 0.2, 0);
                sweepDetected |= backend.sweepKinematic(
                        leg, previousLeg, nextLeg).hit();
                backend.setKinematicTransform(leg, nextLeg, FIXED_DT, true);
                backend.step(FIXED_DT);
                maximumRibbonX = Math.max(maximumRibbonX,
                        backend.getBodyState(bottom).boneTransform().translation()[0]);
                maximumPenetration = Math.max(maximumPenetration,
                        backend.getMaximumPenetration());
                previousLeg = nextLeg;
            }
            return new SpikeReport(backend.getNativeBulletVersion(), FIXED_DT,
                    sweepDetected, maximumRibbonX, maximumPenetration);
        }
    }

    private static RigidBodyBackend.BodyDefinition dynamicBody(
            String name, RigidBodyBackend.Transform initial,
            double hx, double hy, double hz) {
        double smallest = Math.min(hx, Math.min(hy, hz));
        return new RigidBodyBackend.BodyDefinition(name,
                RigidBodyBackend.MotionType.DYNAMIC,
                List.of(box(hx, hy, hz)), initial, 1, 0.45, 0,
                new RigidBodyBackend.CcdSettings(
                        true, smallest * 0.5, smallest * 0.8));
    }

    private static RigidBodyBackend.BodyDefinition kinematicBody(
            String name, RigidBodyBackend.Transform initial,
            double hx, double hy, double hz) {
        return new RigidBodyBackend.BodyDefinition(name,
                RigidBodyBackend.MotionType.KINEMATIC,
                List.of(box(hx, hy, hz)), initial, 0, 0.45, 0,
                RigidBodyBackend.CcdSettings.disabled());
    }

    private static RigidBodyBackend.BoxShape box(double hx, double hy, double hz) {
        return new RigidBodyBackend.BoxShape(new double[]{hx, hy, hz},
                RigidBodyBackend.Transform.identity());
    }

    private static RigidBodyBackend.Transform transform(double x, double y, double z) {
        return new RigidBodyBackend.Transform(new double[]{x, y, z},
                new double[]{0, 0, 0, 1});
    }

    public record SpikeReport(int bulletVersion, double fixedDt,
                              boolean sweepDetected, double maximumRibbonX,
                              double maximumPenetration) {
    }

    public record ModelReport(int sourceBones, int compiledBodies,
                              int compiledBoxes, int skippedDegenerateBoxes,
                              double instantiationMilliseconds) {
    }
}
