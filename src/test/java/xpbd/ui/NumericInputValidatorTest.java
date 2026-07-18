package xpbd.ui;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

final class NumericInputValidatorTest {
    @Test
    void integerInputRejectsBlankFractionalAndOutOfRangeText() {
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseInteger("", 1, 16, "substeps"));
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseInteger("2.5", 1, 16, "substeps"));
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseInteger("17", 1, 16, "substeps"));
        assertEquals(4, NumericInputValidator.parseInteger(" 4 ", 1, 16, "substeps"));
    }

    @Test
    void doubleInputRejectsNonFiniteAndOutOfRangeText() {
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseDouble("NaN", 0, 1, "value"));
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseDouble("Infinity", 0, 1, "value"));
        assertThrows(IllegalArgumentException.class,
                () -> NumericInputValidator.parseDouble("2", 0, 1, "value"));
        assertEquals(0.25,
                NumericInputValidator.parseDouble(".25", 0, 1, "value"), 0);
    }
}
