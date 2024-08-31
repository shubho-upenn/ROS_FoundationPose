# ROS FoundationPose

All other instructions remain the the same (as compared to the main branch)

## Implementation

### 0. Keypoint Annotation
If the keypoints haven't already been anotated (or to modify the annotation of the keypoints) do the following.
```bash
cd FoundationPose_ROS/mesh
python annotate_keypoints.py input_folder  ## python3 doesn't work for this
```
Here, folder_name should be the name of the folder (containing the obj file), present in the current 'mesh' directory.
Press shift and select the keypoints on the GUI and press "Done" when the keypoints are selected.
A file 'keypoints.npy' will be saved inside input_folder

### 1. Run the docker containers 

To run the docker containers, open two separate terminals. In the first terminal run the GroundedSAM ROS container using:
```bash
cd Grounded-Segment-Anything
bash run_gsa_ros.sh
```

In the other terminal run the FoundationPose ROS container using:
```bash
cd FoundationPose_ROS/docker
bash run_container_new_env.sh
```

### 2. Launch the RealSense Node
Launch the RealSense node in a new terminal on the host computer. Be sure to set `align_depth:=true`. 

### 3. Run the GroundedSAM ROS node
In the gsa_ros (GroundedSAM) container do the following to run the ROS GroundedSAM:
```bash
cd Grounded-Segment-Anything
python3 ros_gsam.py
```
Enter the object that you want to track to enable the GroundedSAM to detect and segment the object in the image and press enter. Accurate and general inputs such as 'mustard bottle' or 'orange hand drill' (as in examples) will work well.

### 4. Run the FoundationPose ROS node
Before running the command, place the `.obj` file of the object you want to track in the `mesh` directory. Also, the 'keypoints.npy' file (made from keypoint_annotation.py) should be present in the same folder.
Then, in the ros_fp_new_env (FoundationPose) container, do the following to run the ROS Foundation Pose tracking:
```bash
conda activate test_env
python3 run_KP_tracking.py -in folder_name
```
folder_name is the name of the folder inside the 'mesh' folder (for example: mustard_bottle, sugar_box, pudding_box)



