package xpbd.ui;

/** JavaFX 微调控件和无界面测试共用的纯数值输入校验器。 */
final class NumericInputValidator {
    private NumericInputValidator() {
    }

    static int parseInteger(String text, int minimum, int maximum, String label) {
        String value = requireText(text, label);
        final int parsed;
        try {
            parsed = Integer.parseInt(value);
        } catch (NumberFormatException error) {
            throw new IllegalArgumentException(label + " must be a whole number", error);
        }
        if (parsed < minimum || parsed > maximum) {
            throw new IllegalArgumentException(label + " must be between "
                    + minimum + " and " + maximum);
        }
        return parsed;
    }

    static double parseDouble(String text, double minimum, double maximum,
                              String label) {
        String value = requireText(text, label);
        final double parsed;
        try {
            parsed = Double.parseDouble(value);
        } catch (NumberFormatException error) {
            throw new IllegalArgumentException(label + " must be numeric", error);
        }
        if (!Double.isFinite(parsed)) {
            throw new IllegalArgumentException(label + " must be finite");
        }
        if (parsed < minimum || parsed > maximum) {
            throw new IllegalArgumentException(label + " must be between "
                    + minimum + " and " + maximum);
        }
        return parsed;
    }

    private static String requireText(String text, String label) {
        if (text == null || text.isBlank()) {
            throw new IllegalArgumentException(label + " is required");
        }
        return text.trim();
    }
}
