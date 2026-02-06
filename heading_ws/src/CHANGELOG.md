# Changelog

All notable changes to this project will be documented in this file.

## [1.0.0] - 2026-02-04

### Added
- Initial release of single LiDAR heading and wall distance estimation system
- Ground plane estimation using RANSAC with Tukey-weighted refinement
- Wall plane extraction with vertical constraint
- Roll, pitch, yaw attitude estimation
- Four wall distances calculation (LF, LB, RF, RB)
- Temporal smoothing for stable output
- Quality control and degradation handling
- Comprehensive configuration system
- ROS1/C++ implementation following Google C++ style guide
- Complete logging system
- Launch files and configuration templates
- Comprehensive documentation

### Features
- No IMU dependency - uses only point cloud data
- Forward axis configuration (+X, -X, +Y, -Y)
- Adjustable ROI and filtering parameters
- Statistical outlier removal
- Voxel grid downsampling
- Multi-plane wall extraction
- Parallel wall validation
- Confidence level estimation (HIGH/MEDIUM/LOW)
- Real-time performance (10-20 Hz on typical hardware)

### Performance
- Optimized for ARM64 platforms
- Adaptive downsampling
- Efficient point cloud processing
- Low-latency output (~50-100ms)

### Documentation
- README with complete usage instructions
- Algorithm flow description
- Configuration parameter reference
- Troubleshooting guide
- Coding standards document
