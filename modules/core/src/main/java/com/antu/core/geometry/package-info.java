/**
 * Geometry types, all of them immutable.
 *
 * <h2>Units</h2>
 *
 * Everything in {@code core} and {@code brain} is SI:
 *
 * <ul>
 *   <li>length in <b>metres</b>
 *   <li>angle in <b>radians</b>, normalised to (-pi, pi]
 *   <li>time in <b>seconds</b> for durations; {@link com.antu.core.time.Stamp}
 *       holds nanoseconds
 *   <li>velocity in <b>m/s</b> and <b>rad/s</b>
 * </ul>
 *
 * Hardware rarely agrees. ARCOS reports millimetres and its own angular units;
 * Android reports m/s^2 already but degrees for some sensors. <b>Drivers convert
 * at their boundary</b> so that nothing above them ever has to ask which unit a
 * number is in. That question, asked once too late, is where sign errors and
 * factor-of-1000 bugs come from.
 *
 * <h2>Immutability</h2>
 *
 * The bus hands payloads to subscribers by reference and never copies. Every type
 * here is therefore final with final fields, and operations return new instances.
 */
package com.antu.core.geometry;
