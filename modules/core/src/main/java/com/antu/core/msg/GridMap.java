package com.antu.core.msg;

/**
 * A top-down occupancy grid: what is known to be occupied, free, or unseen.
 * Immutable.
 *
 * <p>Three states rather than two, and the third is the important one. A cell
 * nothing has ever looked at is not free — it is unknown — and a planner that
 * conflates the two will happily route through a wall it has never seen. Most of
 * a map is unknown most of the time.
 *
 * <p>Cells hold a count of supporting observations rather than a boolean, so a
 * single spurious feature point does not create a wall. Confidence accumulates
 * where geometry is seen repeatedly, which is exactly where it should.
 */
public final class GridMap {

    /** Cell has never been observed. */
    public static final byte UNKNOWN = 0;

    private final byte[] hits;
    /** Metres per cell. */
    public final double resolution;
    /** Cells across and down. */
    public final int width;
    public final int height;
    /** World coordinates of the grid's lower-left corner, metres. */
    public final double originX;
    public final double originY;
    /** Observations at or above this count are treated as solid. */
    public final int occupiedThreshold;

    public GridMap(byte[] hits, int width, int height, double resolution,
                   double originX, double originY, int occupiedThreshold) {
        this.hits = hits;
        this.width = width;
        this.height = height;
        this.resolution = resolution;
        this.originX = originX;
        this.originY = originY;
        this.occupiedThreshold = occupiedThreshold;
    }

    /** Observation count for a cell, saturating rather than wrapping. */
    public int at(int cx, int cy) {
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) {
            return UNKNOWN;
        }
        return hits[cy * width + cx] & 0xFF;
    }

    /** True when enough observations support this cell to call it solid. */
    public boolean isOccupied(int cx, int cy) {
        return at(cx, cy) >= occupiedThreshold;
    }

    /** True when nothing has ever been seen here — not the same as free. */
    public boolean isUnknown(int cx, int cy) {
        return at(cx, cy) == UNKNOWN;
    }

    /** Grid column for a world x, which may be outside the grid. */
    public int cellX(double worldX) {
        return (int) Math.floor((worldX - originX) / resolution);
    }

    public int cellY(double worldY) {
        return (int) Math.floor((worldY - originY) / resolution);
    }

    /** World x of a cell's centre. */
    public double worldX(int cx) {
        return originX + (cx + 0.5) * resolution;
    }

    public double worldY(int cy) {
        return originY + (cy + 0.5) * resolution;
    }

    /** How many cells have been observed at all. */
    public int observedCells() {
        int n = 0;
        for (byte b : hits) {
            if (b != UNKNOWN) {
                n++;
            }
        }
        return n;
    }

    /** How many cells are solid enough to plan around. */
    public int occupiedCells() {
        int n = 0;
        for (byte b : hits) {
            if ((b & 0xFF) >= occupiedThreshold) {
                n++;
            }
        }
        return n;
    }

    /** A copy of the raw counts, row-major from the lower-left. */
    public byte[] cells() {
        return hits.clone();
    }

    @Override public String toString() {
        return String.format("GridMap{%dx%d @ %.2fm, %d observed, %d occupied}",
                width, height, resolution, observedCells(), occupiedCells());
    }
}
