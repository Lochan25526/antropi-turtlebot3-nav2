# Third-party models in this directory

`mailbox`, `table_marble`, `cafe_table` and `first_2015_trash_can` are copied unchanged from
https://github.com/osrf/gazebo_models (Apache-2.0). The ROBOTIS `turtlebot3_house` model
references them by `model://` URI; without a local copy Gazebo Classic tries to download them
from the retired online model database and the world takes about three minutes to load.

`moving_obstacle`, `static_box` and `mapper_lidar` are written for this package.
