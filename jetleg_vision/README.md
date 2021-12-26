# jetleg_vision

Vision module for jetleg.

## Note
To run ORB-SLAM2, the ORBvoc.txt file must be manually downloaded and extracted to `jetleg_vision/config`:

```
wget https://github.com/raulmur/ORB_SLAM2/raw/master/Vocabulary/ORBvoc.txt.tar.gz
tar -xvzf ORBvoc.txt.tar.gz
mv ORBvoc.txt config/
rm ORBvoc.txt.tar.gz
```

## Launch SLAM
`ros2 run jetleg_vision jetleg_slam.launch.py`