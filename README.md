## Augmented Reality - Digital drawing

**Algorithm to use for this project:**


   - Camshift algorithm:
        Object tracking, better than meanshift. Tracks the moving object and later used for background subtraction
        It consists of taking the first frame of the video to map its colors with a histogram and then tracks these in the subsequent frames of the video

   - Averaging background:
        Due to changing illumination and other effects, averaging the backgraound is necessary to detect and tract the object

   - Convex hull:
        To find the convex hull of a points set

   - Labeling:
        - connected component labeling and analysis
        - blob extraction
        - region labeling
        - blob extraction
        - region extraction


