#!/bin/bash
# author: sujit-168 su2054552689@gmail.com

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

rm -rf ../../build/
rm -rf ../../devel/
rm -rf ../../install/
rm -rf ../../log/

# build
pushd `pwd` > /dev/null

if [ "$ROS_VERSION" == "2" ]; then
    if [ "$ROS_DISTRO" == "humble" ]; then
        echo "Not use -DBUILD_BEFORE_HUMBLE=ON args for building..."
        cd ../../
        colcon build --symlink-install --continue-on-error
    else
        echo "ROS is not Humble ($ROS_DISTRO) "
        echo "Will use -DBUILD_BEFORE_HUMBLE=ON args for building..."
        cd ../../
        colcon build --symlink-install --cmake-args -DBUILD_BEFORE_HUMBLE=ON --continue-on-error
    fi
else
    exit
fi

popd > /dev/null
