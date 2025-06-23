# Visualization Files

This folder contains visualization and debugging files generated during LIO-SAM testing and calibration.

## Contents

### Frame Visualization Files
- **`.gv` files**: GraphViz source files showing TF frame relationships
- **`.pdf` files**: Rendered PDF visualizations of the frame trees

### File Naming Convention
Files follow the pattern: `frames_YYYY-MM-DD_HH.MM.SS.*`

Example: `frames_2025-06-11_13.42.30.pdf` was generated on June 11, 2025 at 13:42:30

## Purpose

These files were generated during:
- **System debugging**: Understanding coordinate frame relationships
- **Calibration process**: Verifying sensor frame connections
- **IMU integration**: Checking transform chains between LiDAR (`world` frame) and IMU (`imu_link` frame)

## Usage

The PDF files can be opened directly to visualize the TF tree structure. The `.gv` files can be processed with GraphViz tools:

```bash
# Generate PDF from GraphViz file
dot -Tpdf frames_file.gv -o output.pdf

# Generate PNG
dot -Tpng frames_file.gv -o output.png
```

## Key Insights

These visualizations helped identify and resolve:
- Missing transforms between `world` and `imu_link` frames
- Incorrect extrinsic calibration causing IMU "flying around" in RViz
- Proper -85° Z-rotation calibration between LiDAR and IMU coordinate systems

## Related

These files are referenced in the main project documentation and were crucial for achieving the working LIO-SAM configuration with SICK MultiScan 136 and WitMotion HWT905. 