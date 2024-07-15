# Learnings I Got and Notes for them

## 1: What is a Blob

## 2: What is Caffe

## 3: Detection Tensor from `detections = self.net.forward()`

The detections tensor typically has a shape of (1, 1, N, 7), where:

- 1: The first dimension is the batch size (usually 1 in most detection tasks).
- 1: The second dimension is the number of channels (usually 1).
- N: The third dimension represents the number of detections made by the model.
- 7: The fourth dimension represents the number of values for each detection (e.g., [batchId, classId, confidence, x1, y1, x2, y2]).

So we can find the number of detections made using `detections.shape[2]`

## 4: How to index Tensors

For example: `confidence = detections[0, 0, i, 2]`

- `detections[0]`: Accesses the first (and only) batch.
- `detections[0, 0]`: Accesses the first (and only) channel.
- `detections[0, 0, i]`: Accesses the i-th detection.
- `detections[0, 0, i, 2]`: Accesses the third value (index 2) of the i-th detection, which corresponds to the confidence score.

<hr>

- The first two indices [0, 0] are fixed because the batch size and the number of channels are both 1.
- The third index [i] iterates over the number of detections (from 0 to N-1).
- The fourth index [2] specifically accesses the confidence score of the i-th detection.

## 5: Values for Each Detection

For each detected object, the detections tensor typically contains the following values:

- Batch ID: This is often 0 in single image inference scenarios where only one image is processed.

- Class ID: Indicates the class of the detected object. This could be a numerical identifier (e.g., 0 for person, 1 for car, etc.).

- Confidence Score: Represents the confidence or certainty that the detected object belongs to the predicted class. Higher scores indicate higher confidence.
- Bounding Box Coordinates: These are usually four values representing the coordinates of the bounding box around the detected object. These - values are often normalized between 0 and 1 relative to the image dimensions.

	- x: X-coordinate of the top-left corner of the bounding box.
	- y: Y-coordinate of the top-left corner of the bounding box.
	- x1: X-coordinate of the bottom-right corner of the bounding box.
	- y1: Y-coordinate of the bottom-right corner of the bounding box.
	- Additional Parameters: Depending on the network architecture and specific application, additional parameters might be included. These could - include information like keypoints for pose estimation or additional attributes related to the detected object.