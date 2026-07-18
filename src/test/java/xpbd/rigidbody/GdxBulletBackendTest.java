package xpbd.rigidbody;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

final class GdxBulletBackendTest {
    private static final double FIXED_DT = 1.0 / 120.0;

    @Test
    void gravityProducesLinearVelocityWithoutAliasingAngularVelocity() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, -9.8, 0)) {
            RigidBodyBackend.BodyHandle body = backend.createBody(dynamicBody(
                    "falling", transform(0, 2, 0), 0.2, 0.2, 0.2));

            backend.step(FIXED_DT);
            RigidBodyBackend.BodyState state = backend.getBodyState(body);
            double[] linear = state.linearVelocity();
            double[] angular = state.angularVelocity();

            assertTrue(linear[1] < -1e-4, "gravity must produce downward velocity");
            assertArrayEquals(new double[]{0, 0, 0}, angular, 1e-7);
            assertNotSame(linear, angular);
        }
    }

    @Test
    void staticGroundPlaneStopsABoxAtYZero() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, -9.8, 0)) {
            backend.createGroundPlane("ground", 0, 0.5, 0);
            RigidBodyBackend.BodyHandle body = backend.createBody(dynamicBody(
                    "falling", transform(0, 1, 0), 0.2, 0.2, 0.2));

            for (int step = 0; step < 240; step++) backend.step(FIXED_DT);

            double y = backend.getBodyState(body).boneTransform().translation()[1];
            assertEquals(0.2, y, 0.03);
            assertTrue(backend.getContactSnapshots().stream().anyMatch(contact ->
                    contact.bodyA().name().equals("ground")
                            || contact.bodyB().name().equals("ground")));
        }
    }

    @Test
    void nativeLoadsAndOffsetCompoundRoundTripsBoneTransform() {
        RigidBodyBackend.Transform initial = transform(2, 3, 4,
                0, Math.sin(Math.PI / 8), 0, Math.cos(Math.PI / 8));
        RigidBodyBackend.BodyDefinition definition = new RigidBodyBackend.BodyDefinition(
                "asymmetric", RigidBodyBackend.MotionType.DYNAMIC,
                List.of(box(0.2, 0.5, 0.3, -0.8, 0, 0),
                        box(0.4, 0.2, 0.2, 0.6, 0.3, 0)),
                initial, 2, 0.5, 0,
                new RigidBodyBackend.CcdSettings(true, 0.1, 0.08));

        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            RigidBodyBackend.BodyHandle body = backend.createBody(definition);
            RigidBodyBackend.BodyState state = backend.getBodyState(body);

            assertEquals(287, backend.getNativeBulletVersion());
            assertArrayEquals(initial.translation(),
                    state.boneTransform().translation(), 2e-5);
            assertQuaternionEquivalent(initial.rotation(),
                    state.boneTransform().rotation(), 2e-5);
        }
    }

    @Test
    void childBoxSweepFindsBodyCrossedByKinematicCompound() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            RigidBodyBackend.BodyHandle target = backend.createBody(dynamicBody(
                    "target", transform(0, 0, 0), 0.2, 0.8, 0.2));
            RigidBodyBackend.BodyHandle leg = backend.createBody(kinematicBody(
                    "leg", transform(-2, 0, 0), 0.25, 0.6, 0.3));

            RigidBodyBackend.SweepResult sweep = backend.sweepKinematic(
                    leg, transform(-2, 0, 0), transform(2, 0, 0));

            assertTrue(sweep.hit());
            assertTrue(sweep.hitFraction() > 0 && sweep.hitFraction() < 1);
            assertEquals(target.name(), sweep.hitBodyName());
        }
    }

    @Test
    void kinematicRotationPublishesAngularVelocity() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            RigidBodyBackend.BodyHandle leg = backend.createBody(kinematicBody(
                    "leg", transform(0, 0, 0), 0.2, 0.8, 0.2));
            RigidBodyBackend.Transform quarterTurn = transform(0, 0, 0,
                    0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4));

            backend.setKinematicTransform(leg, quarterTurn, 0.1, true);
            double[] angularVelocity = backend.getBodyState(leg).angularVelocity();

            assertEquals(0, angularVelocity[0], 1e-5);
            assertEquals(0, angularVelocity[1], 1e-5);
            assertEquals(Math.PI * 5, angularVelocity[2], 2e-4);
        }
    }

    @Test
    void contactDiagnosticsExposeActualManifoldAndCompoundShapes() {
        try (GdxBulletBackend backend = new GdxBulletBackend(0, 0, 0)) {
            backend.createBody(kinematicBody(
                    "collider", transform(0, 0, 0), 0.5, 0.5, 0.5));
            backend.createBody(dynamicBody(
                    "ribbon", transform(0.7, 0, 0), 0.5, 0.5, 0.5));

            backend.step(FIXED_DT);

            List<RigidBodyBackend.ContactSnapshot> contacts =
                    backend.getContactSnapshots();
            assertTrue(!contacts.isEmpty(), "overlapping boxes must form a manifold");
            RigidBodyBackend.ContactSnapshot contact = contacts.get(0);
            assertTrue(List.of(contact.bodyA().name(), contact.bodyB().name())
                    .containsAll(List.of("collider", "ribbon")));
            assertTrue(contact.penetration() > 0);
            assertEquals(3, contact.pointOnA().length);
            assertEquals(3, contact.pointOnB().length);
            assertEquals(3, contact.normalOnB().length);
            assertEquals(3, contact.relativeTangentVelocityBefore().length);
            assertEquals(3, contact.relativeTangentVelocityAfter().length);
            assertEquals(2, backend.getBodyShapeSnapshots().size());
            assertEquals(0, backend.getBodyShapeSnapshots().get(0).shapeIndex());
        }
    }

    @Test
    void emptyKinematicAnchorJointsToDynamicBodyWithoutCollisionShape() {
        RigidBodyBackend.BodyDefinition anchorDefinition =
                new RigidBodyBackend.BodyDefinition(
                        "empty-anchor", RigidBodyBackend.MotionType.KINEMATIC,
                        List.of(), transform(0, 1, 0), 0, 0.5, 0,
                        RigidBodyBackend.CcdSettings.disabled());
        try (GdxBulletBackend backend = new GdxBulletBackend(0, -9.8, 0)) {
            RigidBodyBackend.BodyHandle anchor =
                    backend.createBody(anchorDefinition);
            RigidBodyBackend.BodyHandle child = backend.createBody(dynamicBody(
                    "child", transform(0, 0, 0), 0.2, 0.5, 0.2));
            RigidBodyBackend.JointSettings joint =
                    new RigidBodyBackend.JointSettings(
                            new double[]{-0.7, -0.7, -0.7},
                            new double[]{0.7, 0.7, 0.7}, 12, 0.8);

            backend.addSpringJoint(anchor, child, transform(0, 0.5, 0), joint);
            backend.step(FIXED_DT);

            assertEquals(1, backend.getBodyShapeSnapshots().size(),
                    "the empty anchor must not contribute collision geometry");
            assertEquals("child",
                    backend.getBodyShapeSnapshots().get(0).body().name());
            assertEquals(1, backend.getBodyState(anchor)
                    .boneTransform().translation()[1], 1e-6);
        }
    }

    @Test
    void fixedStepKickOfJointedRibbonIsDeterministic() {
        KickResult first = runKickScenario();
        KickResult second = runKickScenario();

        assertTrue(first.sweepDetected, "kinematic child sweep should see the ribbon");
        assertTrue(first.maximumRibbonX > 0.05,
                "moving leg should kick the ribbon in its travel direction");
        assertTrue(first.maximumPenetration < 0.3,
                "deep-penetration audit exceeded the spike threshold");
        assertEquals(first.maximumRibbonX, second.maximumRibbonX, 1e-6);
        assertArrayEquals(first.finalTranslation, second.finalTranslation, 1e-6);
        assertArrayEquals(first.finalRotation, second.finalRotation, 1e-6);
    }

    private static KickResult runKickScenario() {
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
                RigidBodyBackend.BodyState bottomState = backend.getBodyState(bottom);
                maximumRibbonX = Math.max(maximumRibbonX,
                        bottomState.boneTransform().translation()[0]);
                maximumPenetration = Math.max(maximumPenetration,
                        backend.getMaximumPenetration());
                previousLeg = nextLeg;
            }

            RigidBodyBackend.BodyState finalState = backend.getBodyState(bottom);
            return new KickResult(sweepDetected, maximumRibbonX, maximumPenetration,
                    finalState.boneTransform().translation(),
                    finalState.boneTransform().rotation());
        }
    }

    private static RigidBodyBackend.BodyDefinition dynamicBody(
            String name, RigidBodyBackend.Transform initial,
            double hx, double hy, double hz) {
        return new RigidBodyBackend.BodyDefinition(name,
                RigidBodyBackend.MotionType.DYNAMIC,
                List.of(box(hx, hy, hz, 0, 0, 0)), initial,
                1, 0.45, 0,
                new RigidBodyBackend.CcdSettings(true,
                        Math.min(hx, Math.min(hy, hz)) * 0.5,
                        Math.min(hx, Math.min(hy, hz)) * 0.8));
    }

    private static RigidBodyBackend.BodyDefinition kinematicBody(
            String name, RigidBodyBackend.Transform initial,
            double hx, double hy, double hz) {
        return new RigidBodyBackend.BodyDefinition(name,
                RigidBodyBackend.MotionType.KINEMATIC,
                List.of(box(hx, hy, hz, 0, 0, 0)), initial,
                0, 0.45, 0, RigidBodyBackend.CcdSettings.disabled());
    }

    private static RigidBodyBackend.BoxShape box(
            double hx, double hy, double hz, double x, double y, double z) {
        return new RigidBodyBackend.BoxShape(new double[]{hx, hy, hz},
                transform(x, y, z));
    }

    private static RigidBodyBackend.Transform transform(double x, double y, double z) {
        return transform(x, y, z, 0, 0, 0, 1);
    }

    private static RigidBodyBackend.Transform transform(
            double x, double y, double z, double qx, double qy, double qz, double qw) {
        return new RigidBodyBackend.Transform(new double[]{x, y, z},
                new double[]{qx, qy, qz, qw});
    }

    private static void assertQuaternionEquivalent(
            double[] expected, double[] actual, double tolerance) {
        double dot = 0;
        for (int i = 0; i < 4; i++) dot += expected[i] * actual[i];
        double sign = dot < 0 ? -1 : 1;
        for (int i = 0; i < 4; i++) {
            assertEquals(expected[i], actual[i] * sign, tolerance);
        }
    }

    private record KickResult(boolean sweepDetected, double maximumRibbonX,
                              double maximumPenetration, double[] finalTranslation,
                              double[] finalRotation) {
    }
}
