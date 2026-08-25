package com.antu.core;

/**
 * A test harness small enough to have no dependencies.
 *
 * <p>The pure modules deliberately have no third-party jars, and adding JUnit to
 * a no-Gradle build to get assertEquals is a poor trade. If the suite outgrows
 * this, that is the moment to reconsider.
 */
public final class Check {

    private final String suite;
    private int checks;
    private int failures;

    public Check(String suite) {
        this.suite = suite;
        System.out.println("== " + suite);
    }

    public void eq(String label, Object expected, Object actual) {
        checks++;
        boolean ok = expected == null ? actual == null : expected.equals(actual);
        if (ok) {
            System.out.println("  ok    " + label);
        } else {
            failures++;
            System.out.println("  FAIL  " + label);
            System.out.println("          expected: " + expected);
            System.out.println("          actual:   " + actual);
        }
    }

    public void pass(String label) {
        checks++;
        System.out.println("  ok    " + label);
    }

    public void fail(String label) {
        checks++;
        failures++;
        System.out.println("  FAIL  " + label);
    }

    /** Prints the tally and exits non-zero if anything failed, so the build stops. */
    public void finish() {
        System.out.println();
        if (failures == 0) {
            System.out.println(suite + ": PASS - " + checks + " checks");
        } else {
            System.out.println(suite + ": FAIL - " + failures + " of " + checks + " failed");
            System.exit(1);
        }
    }
}
