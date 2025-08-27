#Edited from https://answers.ros.org/question/300113/docker-how-to-use-rviz-and-gazebo-from-a-container/
# https://docs.ros.org/en/humble/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html
# sudo bash ros2-humble-docker.sh
# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

XAUTH=/tmp/.docker.xauth
USER=antonio 

echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."
docker run -it\
    --rm\
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/$USER/Desktop/navigation2/nav2/params:/opt/ros/humble/share/nav2_bringup/params" \
    --volume="/home/$USER/Desktop/navigation2/maps:/opt/ros/humble/share/nav2_bringup/maps" \
    --volume="/home/$USER/Desktop/navigation2/rviz_config:/rviz_config" \
    --volume="/home/$USER/Desktop/navigation2/robot_localization/params:/opt/ros/humble/share/robot_localization/params" \
    --volume="/home/$USER/Desktop/navigation2/behavior_trees:/opt/ros/humble/share/nav2_bringup/behavior_trees" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --pid=host \
    --ipc=host \
    --name=ros-utilities \
    -t matheusdutra0207/ros-utilities:0.0.1 \
    bash
echo "Done."
