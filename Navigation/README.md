# Navigation
## Folders
1. `ArucoDetect` - This folder contains the main Navigation subroutine (C++ source code for detecting ArUco markers).

2. `ArucoDetect1Cam` - This folder contains the C++ source code to detect ArUCo markers for 1 camera. It is just for debugging purposes.

3. `CreateMarkers` - This folder contains the C++ source code to generate ArUCO marker.

## Files
1. `ArucoDetect_Old.cpp` has the C++ source code for pose estimation of ArUCo markers using a constant derived from mapping centimeters to pixels at a camera-to-platform distance of 235 cm. If the distance between camera and platform changes, the constant to map pixels to cm changes. Hence, this method was discarded.
    #### Inputs:
      1. **`cameraMatrix1`**: Intrinsic matrix of camera 1.
      2. **`cameraMatrix2`**: Intrinsic matrix of camera 2.
      3. **`distCoeffs1`**: Distortion coefficients of camera 1.
      4. **`distCoeffs2`**: Distortion coefficients of camera 2.
      5. **`markerLength`**: Length of ArUCo marker in same units as the size of square in checkerboard mentioned during camera calibration.
      6. **`height`**: (In `centerCordinatesInCms_Cam1` and `centerCordinatesInCms_Cam2`) Distance between camera and platform.
      7. **`calibrationConstant`**: (In `centerCordinatesInCms_Cam1` and `centerCordinatesInCms_Cam2`) Mapping constant between pixels and centimeters (cm/pixel).

    #### Outputs:
      1. **`finalCoordinates`**: The final (x,y) coordinates of both spacecraft relative to inertial marker.
      2. **`finalZAngle`**: The final yaw angle of both spacecraft relative to inertial marker.
         

2. `ArucoDetect_ROSCode.cpp` has ROS C++ version of `ArucoDetect_Old.cpp`. In this code, ROS initializes the **`Position`** node which publishes two topics: **`TargetPosition`** and **`ChaserPosition`**, of type `geometry_msgs/Vector3`.

3. `CalibrateCamera.cpp` contains the C++ source code for calibrating the camera. When the images of a checkerboard taken by the camera are provided, this subroutine calculates the Intrinsic matrix and distortion coefficients of the camera.
    #### Inputs:
      1. **`fileNames`**: Path to the images folder.
      2. **`patternSize`**: Size of the checkerboard ((rows, cols) - (1, 1)).
      3. **`checkerBoard`**: Size of the checkerboard ({rows, cols}).
      4. **`fieldSize`**: Size of each square in the checkerboard.

    #### Outputs:
      1. **`K`**: Intrinsic matrix of the camera.
      2. **`k`**: Distortion coefficients of the camera.
