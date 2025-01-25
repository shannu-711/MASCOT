# ArucoDetect1Cam
The file `ArucoDetect1Cam` contains the C++ source code to process frames from a single camera and detect the ArUCo markers. The code returns the center coordinates of the markers detected.
    #### Inputs:
      1. **`cameraMatrix1`**: Intrinsic matrix of camera.
      3. **`distCoeffs1`**: Distortion coefficients of camera.
      5. **`markerLength`**: Length of ArUCo marker in same units as the size of square in checkerboard mentioned during camera calibration.
    #### Outputs:
      1. **`centers1`**: (x, y) coordinates of the center pixel of detected ArUCo markers.
