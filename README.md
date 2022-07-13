# summer-project

First create an virtual environment with python==2.7
```
conda env create -f environment.yaml
```
Activate virtual environment: name "ros"
```
conda activate ros
```
Start ros
```
roscore
```
Open Rviz
```
rviz
```
Publish a static transform
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 map base_link 50
```
Run python
```
python semantic_map_publisher.py
```
