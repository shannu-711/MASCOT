# CreateMarkers
`CreateMarkers.cpp` contains the C++ source code to generate ArUCO marker of a specified size and ID from the **DICT_6X6_50** dictionary and save the image in the directory.

The image is saved as `marker{ID}.png` (e.g., marker0.png).

#### Inputs:
  1. **`dictionary`**: ArUCo marker dictionary from which to generate the marker.
  2. **`ID`**: ID of the ArUCo marker to generate.
  3. **`markersizeInPixels`**: Desired size of the marker in pixels.

#### Outputs:
  1. **`markerImage`**: Generated image of the ArUCo marker.
