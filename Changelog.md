# Change Log

----------------------------
### 2022.1.9
#### REVISE: Feature
#### Purpose: Optimize memory
#### Details: 
    The previous version used a one-dimensional vector to save keypoints, which made it impossible for NMS suppressed points and unnecessary points(restrict from maximum number of keypoints) to be eliminated without compromising efficiency. Now save the keypoints through a 2D vector, let the score of the points suppressed by NMS set to -1, and then directly eliminate these points by sorting and resize without compromising efficiency.

--------------------------
### 2022.1.18
#### ADD: Descriptor
#### Purpose: Add Brief descriptor
#### Details:
    Implement the brief algorithm.

--------------------------
### 2022.1.18
#### REVISE: Replace uintX_t with int
#### Purpose: Reduce possible future bugs
#### Details:
     At first, I used uintX_t to represent integer data, this is realistic. For example, rows and cols of image, they can't be negative. However, This gives unexpected error，such as when I compare unsigned int and int. So I replace uintX_t with int as far as I can.
     I tested the revise and passed.

--------------------------
### 2022.1.20
#### REVISE: Remove template in class Descriptor
#### Purpose: Keep the interface separate from the implementation
#### Details:
    It is impossible for us to know the type of data used in the future, so I want to generic programming using templates. But if I do that, I can't maintain the interface separate from the implementation. 
    So I gave it up, but I would implement the lib using template in the future.

---------------------------
#### ADD: Add class Descriptor test
#### Purpose: Simple test for brief
      
