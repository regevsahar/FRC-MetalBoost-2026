package frc.lib.util.MapFiltering;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * GridMap represents a 2D occupancy grid of the FRC field.
 *
 * <p>The grid is indexed from the <b>bottom-left</b> corner of the field:
 *
 * <ul>
 *   <li>Row 0 corresponds to the bottom of the field
 *   <li>Column 0 corresponds to the left side of the field
 * </ul>
 *
 * <p>Each grid cell represents a square area of {@code nodeSizeMeters × nodeSizeMeters}. A value of
 * {@code true} indicates a blocked / red / forbidden cell.
 *
 * <p>All conversions from world coordinates (meters) to grid indices use {@link Math#floor(double)}
 * to ensure consistent indexing.
 */
public class GridMap {

  /** Occupancy grid: true = blocked, false = free */
  private final boolean[][] grid;

  /** Size of one grid cell in meters */
  private final double nodeSizeMeters;

  /** Full field width in meters */
  private final double fieldWidthMeters;

  /** Full field height in meters */
  private final double fieldHeightMeters;

  /** Number of rows in the grid */
  private final int rows;

  /** Number of columns in the grid */
  private final int cols;

  /**
   * Creates a new GridMap.
   *
   * @param grid 2D boolean occupancy grid (true = blocked)
   * @param nodeSizeMeters size of one grid cell in meters
   * @param fieldWidthMeters full field width in meters
   * @param fieldHeightMeters full field height in meters
   */
  public GridMap(
      boolean[][] grid, double nodeSizeMeters, double fieldWidthMeters, double fieldHeightMeters) {
    this.grid = grid;
    this.nodeSizeMeters = nodeSizeMeters;
    this.fieldWidthMeters = fieldWidthMeters;
    this.fieldHeightMeters = fieldHeightMeters;

    this.rows = grid.length;
    this.cols = grid[0].length;
  }

  /* ===============================
  Public API
  =============================== */

  /**
   * Checks whether a pose is located in a blocked (red) grid cell.
   *
   * @param pose robot pose in field coordinates (meters)
   * @return true if the pose lies in a blocked cell or outside the grid
   */
  public boolean onForbiddenArea(Pose2d pose) {
    GridIndex index = poseToGrid(pose);
    return onForbiddenArea(index);
  }

  /**
   * Checks whether a grid index corresponds to a blocked cell.
   *
   * @param index grid index
   * @return true if blocked or outside the grid
   */
  public boolean onForbiddenArea(GridIndex index) {
    if (!isInsideGrid(index)) {
      return true; // Outside the field is treated as blocked
    }
    return grid[index.row()][index.col()];
  }

  /**
   * Converts a {@link Pose2d} in meters to a grid index.
   *
   * @param pose robot pose in meters
   * @return grid index corresponding to the pose
   */
  public GridIndex poseToGrid(Pose2d pose) {
    return metersToGrid(pose.getX(), pose.getY());
  }

  /**
   * Converts field coordinates in meters to grid indices. The conversion uses {@link
   * Math#floor(double)} for both axes.
   *
   * @param xMeters x-position in meters (left → right)
   * @param yMeters y-position in meters (bottom → top)
   * @return grid index corresponding to the given coordinates
   */
  public GridIndex metersToGrid(double xMeters, double yMeters) {
    int col = (int) Math.floor(xMeters / nodeSizeMeters);
    int row = (int) Math.floor(yMeters / nodeSizeMeters);

    return new GridIndex(row, col);
  }

  /**
   * Converts a grid index back to a {@link Pose2d} located at the center of the grid cell.
   *
   * @param index grid index
   * @return pose at the center of the specified grid cell
   */
  public Pose2d gridToPose(GridIndex index) {
    double x = (index.col() + 0.5) * nodeSizeMeters;
    double y = (index.row() + 0.5) * nodeSizeMeters;
    return new Pose2d(x, y, new Rotation2d());
  }

  /* ===============================
  Helper Methods
  =============================== */

  /**
   * Checks whether a grid index lies within the grid bounds.
   *
   * @param index grid index
   * @return true if the index is inside the grid
   */
  public boolean isInsideGrid(GridIndex index) {
    return index.row() >= 0 && index.row() < rows && index.col() >= 0 && index.col() < cols;
  }

  /**
   * @return number of grid rows
   */
  public int getRows() {
    return rows;
  }

  /**
   * @return number of grid columns
   */
  public int getCols() {
    return cols;
  }
}
