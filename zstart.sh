gnome-terminal -- bash -c "
cd ~/ros_ws/navigation_sim_origin;
source install/setup.bash;
export __NV_PRIME_RENDER_OFFLOAD=1;
export __GLX_VENDOR_LIBRARY_NAME=nvidia;
ros2 launch bringup startup.launch.py;
exec bash;
"
