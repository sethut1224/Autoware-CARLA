SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

sudo apt-key del  F42ED6FBAB17C654
sudo -E apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

sudo apt-get install --no-install-recommends -y \
    python$PYTHON_SUFFIX-pip \
    ros-$ROS_DISTRO-ackermann-msgs \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-derived-object-msgs \
    ros-$ROS_DISTRO-pcl-conversions \

pip$PYTHON_SUFFIX install --upgrade pip$PYTHON_SUFFIX
pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/requirements.txt

echo export CARLA_AUTOWARE_CONTENTS=$HOME/shared_dir/data/carla/ >> ~/.bashrc
echo export PYTHONPATH='$PYTHONPATH:$HOME/shared_dir/CARLA/PythonAPI/carla/dist/carla-0.9.10-py2.7-linux-x86_64.egg' >> ~/.bashrc

