# Change Log

----------------------------
### 2022.1.9
#### REVISE: Feature
#### Purepose: optimize memory
#### Details: 
    The previous version used a one-dimensional vector to save keypoints, which made it impossible for NMS suppressed points and unnecessary points(restrict from maximum number of keypoints) to be eliminated without compromising efficiency. Now save the keypoints through a 2D vector, let the score of the points suppressed by NMS set to -1, and then directly eliminate these points by sorting and resize without compromising efficiency.
--------------------------
