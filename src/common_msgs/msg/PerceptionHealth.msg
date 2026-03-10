# PerceptionHealth.msg
# ARM-optimized health status message for on-demand queries
# Total fixed payload: 48 bytes (12 x 4-byte words)
# Optimized for Raspberry Pi 5 and Jetson Orin Nano

# ====== HEADER (8 bytes) ======
builtin_interfaces/Time stamp

# ====== 4-BYTE FLOAT GROUP (32 bytes total) ======
# All float32 fields grouped for ARM cache efficiency
float32 overall_health          # System health score (0.0 to 1.0)
float32 lio_health              # LIO estimator health
float32 vio_health              # VIO estimator health
float32 kinematic_health        # Kinetic estimator health
float32 lidar_health            # LiDAR sensor health
float32 camera_health           # Camera sensor health
float32 imu_health              # IMU sensor health
float32 time_sync_skew_ms       # Current max skew in milliseconds

# ====== 1/2-BYTE METADATA GROUP (4 bytes total) ======
# Small fields packed together to minimize padding
uint16 fault_flags              # Fault bitmask (see constants below)
uint8 mode                      # Mode enum (see constants below)
bool time_sync_valid            # Within thresholds?
bool service_responsive         # Health service responding?

# ====== VARIABLE-LENGTH STRINGS (at end - don't affect alignment) ======
string mode_string              # Human-readable mode
string[] active_faults          # List of active fault names

# ============================================================================
# FAULT FLAGS BITMASK (uint16) - 10 fault types for prototyping
# ============================================================================
uint16 FAULT_NONE = 0              # Bit 0: No faults
uint16 FAULT_LIO_STALE = 1         # Bit 0: LIO not publishing
uint16 FAULT_VIO_STALE = 2         # Bit 1: VIO not publishing
uint16 FAULT_KINEMATIC_STALE = 4   # Bit 2: Kinetic not publishing
uint16 FAULT_LIDAR_STALE = 8       # Bit 3: LiDAR sensor stale
uint16 FAULT_CAMERA_STALE = 16     # Bit 4: Camera sensor stale
uint16 FAULT_IMU_STALE = 32        # Bit 5: IMU sensor stale
uint16 FAULT_TIME_SYNC = 64        # Bit 6: Time sync violation
uint16 FAULT_COVARIANCE_EXCEEDED = 128  # Bit 7: High uncertainty
uint16 FAULT_CALIBRATION_INVALID = 256  # Bit 8: Bad calibration data
uint16 FAULT_SERVICE_UNAVAILABLE = 512  # Bit 9: Health service down

# ============================================================================
# MODE ENUM (uint8)
# ============================================================================
uint8 MODE_NOMINAL = 0             # Health >= 0.75
uint8 MODE_DEGRADED_LIO = 1        # LIO health < 0.75
uint8 MODE_DEGRADED_VIO = 2        # VIO health < 0.75
uint8 MODE_SAFE_STOP = 3           # Overall health < 0.55
