# MANUS SDK Bridge

This package builds the real MANUS glove publisher from the official MANUS SDK.
The SDK shared libraries are not stored in this repository; install them with:

```bash
MANUS_SDK_ARCHIVE=/path/to/MANUS_SDK.zip ../../scripts/install_manus_sdk.sh
```

If you already have the official MANUS SDK files on a development machine, make
a customer install archive from the workspace root:

```bash
./scripts/package_manus_sdk.sh
```

Expected local layout after installation:

```text
src/manus_ros2/ManusSDK/include/ManusSDK.h
src/manus_ros2/ManusSDK/lib/libManusSDK.so
src/manus_ros2/ManusSDK/lib/libManusSDK_Integrated.so
```

When the SDK is absent, `colcon build` still succeeds for this package but skips
the `manus_data_publisher` and `manus_calibration_tool` executables.
