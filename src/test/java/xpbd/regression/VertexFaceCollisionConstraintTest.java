package xpbd.regression;

import constraints.VertexFaceCollisionConstraint;
import models.Particle;
import models.Vector3;
import org.junit.jupiter.api.Test;
import xpbd.baker.BodyColliderCache;
import xpbd.baker.BonePoseCalculator;
import xpbd.loader.BedrockAnimationData;
import xpbd.loader.BedrockModelData;

import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

final class VertexFaceCollisionConstraintTest {
    private static final double DT = 1.0 / 60.0;
    private static final double SKIN = 0.1;

    @Test
    void cubeBuildsSixUnitOutwardPlanes() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        BodyColliderCache.Collider collider = fixture.cache.getColliders().get(0);
        double[][] expected = {
                {-1, 0, 0}, {1, 0, 0}, {0, -1, 0},
                {0, 1, 0}, {0, 0, -1}, {0, 0, 1}
        };
        for (int face = 0; face < 6; face++) {
            assertEquals(expected[face][0], collider.getBindNormal(face, 0), 1e-12);
            assertEquals(expected[face][1], collider.getBindNormal(face, 1), 1e-12);
            assertEquals(expected[face][2], collider.getBindNormal(face, 2), 1e-12);
        }
    }

    @Test
    void insidePointUsesDeterministicNearestExitWithoutBounce() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = particle(0, 0, 0, 0, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint();

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertTrue(particle.getPosition().x < -1.0999);
        assertEquals(0, particle.getPosition().y, 1e-12);
        assertEquals(0, particle.getVelocity().x, 1e-9);
        assertFalse(fixture.cache.containsCurrent(particle.getPosition().x,
                particle.getPosition().y, particle.getPosition().z, SKIN));
    }

    @Test
    void outsidePointIsStrictNoOp() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = particle(1.5, 0, 0, 1.5, 0, 0);

        fixture.constraint().solve(new Particle[]{particle}, DT);

        assertArrayEquals(new double[]{1.5, 0, 0}, vector(particle.getPosition()), 0);
    }

    @Test
    void tangentialVelocitySurvivesDeepProjection() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = particle(0, 0, 0, 0, -0.1, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint();

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertEquals(0, particle.getVelocity().x, 1e-9);
        assertEquals(6, particle.getVelocity().y, 1e-9);
    }

    @Test
    void highSpeedOutsideToOutsideSweepStopsAtEntryFace() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        fixture.cache.advance(fixture.poses, true);
        Particle particle = particle(2, 0, 0, -2, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint();

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertTrue(particle.getPosition().x < -1.0999);
        assertEquals(0, particle.getVelocity().x, 1e-8);
        assertEquals(1, constraint.getDiagnostics().sweepHits);
    }

    @Test
    void highSpeedOutsideToInsideStopsAtEntryFaceWithoutEnergyGain() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        fixture.cache.advance(fixture.poses, true);
        Particle particle = particle(0, 0, 0, 2, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint();

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertTrue(particle.getPosition().x > 1.0999,
                "the particle must remain on the +X entry side");
        assertEquals(0, particle.getVelocity().x, 1e-8);
        assertEquals(1, constraint.getDiagnostics().sweepHits);
    }

    @Test
    void incomingVelocityUsesConfiguredRestitution() {
        assertStaticImpactRestitution(0.5, -120);
        assertStaticImpactRestitution(1.0, -240);
    }

    @Test
    void separatingNormalVelocityIsPreserved() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = particle(0.9, 0, 0, 0.8, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint(1.0);

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertEquals(6, particle.getVelocity().x, 1e-8);
    }

    @Test
    void initialEmbeddingNeverCreatesBounce() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = particle(0, 0, 0, 0, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint(1.0);

        constraint.projectInitial(new Particle[]{particle});
        constraint.postSolveVelocity(new Particle[]{particle});

        assertFalse(fixture.cache.containsCurrent(particle.getPosition().x,
                particle.getPosition().y, particle.getPosition().z, SKIN));
        assertArrayEquals(new double[]{0, 0, 0}, vector(particle.getVelocity()), 0);
    }

    @Test
    void restitutionMustBeFiniteAndWithinUnitRange() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));

        assertThrows(IllegalArgumentException.class,
                () -> fixture.constraint(-0.01));
        assertThrows(IllegalArgumentException.class,
                () -> fixture.constraint(1.01));
        assertThrows(IllegalArgumentException.class,
                () -> fixture.constraint(Double.NaN));
    }

    @Test
    void overlappingCubesChooseUnionExit() {
        Fixture fixture = fixture(List.of(
                cube(-1, -1, -1, 2, 2, 2),
                cube(-0.5, -1, -1, 2, 2, 2)));
        Particle particle = particle(0, 0, 0, 0, 0, 0);

        fixture.constraint().solve(new Particle[]{particle}, DT);

        assertFalse(fixture.cache.containsCurrent(particle.getPosition().x,
                particle.getPosition().y, particle.getPosition().z, SKIN));
        assertTrue(particle.getPosition().x > 1.0999);
    }

    @Test
    void fixedPointInsideIsReportedButNotMoved() {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        Particle particle = new Particle(0);
        particle.setPosition(new Vector3());
        particle.setPrevPosition(new Vector3());
        VertexFaceCollisionConstraint constraint = fixture.constraint();

        constraint.solve(new Particle[]{particle}, DT);

        assertArrayEquals(new double[]{0, 0, 0}, vector(particle.getPosition()), 0);
        assertEquals(1, constraint.getDiagnostics().fixedInside);
    }

    @Test
    void movingBodyCarriesSwallowedPointWithoutBounce() {
        BedrockModelData.Bone body = new BedrockModelData.Bone();
        body.name = "body";
        body.cubes.add(cube(-1, -1, -1, 2, 2, 2));
        List<BedrockModelData.Bone> bones = List.of(body);
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        BedrockAnimationData.BoneAnimation bodyAnimation =
                new BedrockAnimationData.BoneAnimation();
        bodyAnimation.position = new BedrockAnimationData.Keyframes();
        bodyAnimation.position.keyframes.put(0.0, new double[]{0, 0, 0});
        bodyAnimation.position.keyframes.put(1.0, new double[]{1, 0, 0});
        animation.bones.put("body", bodyAnimation);
        BodyColliderCache cache = new BodyColliderCache(bones, Set.of("body"));
        cache.initialize(BonePoseCalculator.calculate(bones, animation, 0));
        cache.advance(BonePoseCalculator.calculate(bones, animation, 1), true);
        Particle particle = particle(-1.5, 0, 0, -1.5, 0, 0);
        VertexFaceCollisionConstraint constraint = new VertexFaceCollisionConstraint(
                new int[]{0}, 1, cache, SKIN);

        constraint.solve(new Particle[]{particle}, 1.0);
        particle.getVelocity().set(
                particle.getPosition().x - particle.getPrevPosition().x,
                particle.getPosition().y - particle.getPrevPosition().y,
                particle.getPosition().z - particle.getPrevPosition().z);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertTrue(particle.getPosition().x < -2.0999);
        assertEquals(-1, particle.getVelocity().x, 1e-9);
    }

    @Test
    void movingBodyBounceUsesVelocityRelativeToFace() {
        BedrockModelData.Bone body = new BedrockModelData.Bone();
        body.name = "body";
        body.cubes.add(cube(-1, -1, -1, 2, 2, 2));
        List<BedrockModelData.Bone> bones = List.of(body);
        BedrockAnimationData.Animation animation = new BedrockAnimationData.Animation();
        BedrockAnimationData.BoneAnimation bodyAnimation =
                new BedrockAnimationData.BoneAnimation();
        bodyAnimation.position = new BedrockAnimationData.Keyframes();
        bodyAnimation.position.keyframes.put(0.0, new double[]{0, 0, 0});
        bodyAnimation.position.keyframes.put(1.0, new double[]{1, 0, 0});
        animation.bones.put("body", bodyAnimation);
        BodyColliderCache cache = new BodyColliderCache(bones, Set.of("body"));
        cache.initialize(BonePoseCalculator.calculate(bones, animation, 0));
        cache.advance(BonePoseCalculator.calculate(bones, animation, 1), true);
        Particle particle = particle(-1.5, 0, 0, -1.5, 0, 0);
        VertexFaceCollisionConstraint constraint = new VertexFaceCollisionConstraint(
                new int[]{0}, 1, cache, SKIN, 0.5);

        constraint.solve(new Particle[]{particle}, 1.0);
        particle.getVelocity().set(
                particle.getPosition().x - particle.getPrevPosition().x,
                particle.getPosition().y - particle.getPrevPosition().y,
                particle.getPosition().z - particle.getPrevPosition().z);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertEquals(-1.5, particle.getVelocity().x, 1e-9);
    }

    private static void assertStaticImpactRestitution(double restitution,
                                                      double expectedVelocityX) {
        Fixture fixture = fixture(List.of(cube(-1, -1, -1, 2, 2, 2)));
        fixture.cache.advance(fixture.poses, true);
        Particle particle = particle(2, 0, 0, -2, 0, 0);
        VertexFaceCollisionConstraint constraint = fixture.constraint(restitution);

        constraint.solve(new Particle[]{particle}, DT);
        rebuildVelocity(particle);
        constraint.postSolveVelocity(new Particle[]{particle});

        assertEquals(expectedVelocityX, particle.getVelocity().x, 1e-8);
    }

    private static Fixture fixture(List<BedrockModelData.Cube> cubes) {
        BedrockModelData.Bone body = new BedrockModelData.Bone();
        body.name = "body";
        body.cubes.addAll(cubes);
        List<BedrockModelData.Bone> bones = List.of(body);
        Map<String, BonePoseCalculator.Pose> poses =
                BonePoseCalculator.calculate(bones, null, 0);
        BodyColliderCache cache = new BodyColliderCache(bones, Set.of("body"));
        cache.initialize(poses);
        return new Fixture(cache, poses);
    }

    private static BedrockModelData.Cube cube(double x, double y, double z,
                                               double sx, double sy, double sz) {
        BedrockModelData.Cube cube = new BedrockModelData.Cube();
        cube.origin = new double[]{x, y, z};
        cube.size = new double[]{sx, sy, sz};
        return cube;
    }

    private static Particle particle(double x, double y, double z,
                                     double previousX, double previousY,
                                     double previousZ) {
        Particle particle = new Particle(1);
        particle.setPosition(new Vector3(x, y, z));
        particle.setPrevPosition(new Vector3(previousX, previousY, previousZ));
        return particle;
    }

    private static void rebuildVelocity(Particle particle) {
        particle.getVelocity().set(
                (particle.getPosition().x - particle.getPrevPosition().x) / DT,
                (particle.getPosition().y - particle.getPrevPosition().y) / DT,
                (particle.getPosition().z - particle.getPrevPosition().z) / DT);
    }

    private static double[] vector(Vector3 value) {
        return new double[]{value.x, value.y, value.z};
    }

    private record Fixture(BodyColliderCache cache,
                           Map<String, BonePoseCalculator.Pose> poses) {
        VertexFaceCollisionConstraint constraint() {
            return new VertexFaceCollisionConstraint(
                    new int[]{0}, 1, cache, SKIN);
        }

        VertexFaceCollisionConstraint constraint(double restitution) {
            return new VertexFaceCollisionConstraint(
                    new int[]{0}, 1, cache, SKIN, restitution);
        }
    }
}
