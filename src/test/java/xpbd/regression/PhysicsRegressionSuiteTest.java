package xpbd.regression;

import org.junit.jupiter.api.Test;

/** Makes the existing dependency-light regression program part of standard Maven tests. */
final class PhysicsRegressionSuiteTest {
    @Test
    void syntheticRegressionSuite() throws Exception {
        PhysicsRegressionTests.main(new String[0]);
    }
}
