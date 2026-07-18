package constraints;

import models.Particle;
import models.Vector3;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

final class GroundCollisionConstraintTest {
    @Test
    void dynamicParticleIsProjectedToSkinAndBounces() {
        Particle particle = new Particle(1);
        particle.setPrevPosition(new Vector3(0, 0.2, 0));
        particle.setPosition(new Vector3(0, -0.1, 0));
        GroundCollisionConstraint constraint = new GroundCollisionConstraint(
                new int[]{0}, 1, 0, 0.05, 0.5);

        constraint.solve(new Particle[]{particle}, 0.1);
        particle.setVelocity(new Vector3(0, -1.5, 0));
        constraint.postSolveVelocity(new Particle[]{particle});

        assertEquals(0.05, particle.getPosition().y, 1e-12);
        assertEquals(1.5, particle.getVelocity().y, 1e-12);
    }

    @Test
    void initialProjectionPreservesHistoryAndDoesNotMoveFixedParticle() {
        Particle dynamic = new Particle(1);
        dynamic.setPosition(new Vector3(0, -1, 0));
        dynamic.setPrevPosition(new Vector3(0, -1.25, 0));
        Particle fixed = new Particle(0);
        fixed.setPosition(new Vector3(0, -2, 0));
        GroundCollisionConstraint constraint = new GroundCollisionConstraint(
                new int[]{0, 1}, 2, 0, 0.1, 0);

        constraint.projectInitial(new Particle[]{dynamic, fixed});

        assertEquals(0.1, dynamic.getPosition().y, 1e-12);
        assertEquals(-0.15, dynamic.getPrevPosition().y, 1e-12);
        assertEquals(0, dynamic.getVelocity().y, 1e-12);
        assertEquals(-2, fixed.getPosition().y, 1e-12);
    }
}
