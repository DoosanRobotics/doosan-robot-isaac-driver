#!/bin/bash
##


image_tag="x86"
isaac_sim_version=""
input_arg="$1"

if [ -z "$input_arg" ]; then
    echo "No input argument detected, defaulting to isaac_sim_4.2.0"
    input_arg="isaac_sim_4.2.0"
fi

if [ "$input_arg" == "isaac_sim_4.2.0" ]; then
    echo "Building Isaac Sim headless docker"
    dockerfile="isaac_sim.dockerfile"
    image_tag="isaac_sim_4.2.0"
    isaac_sim_version="4.2.0"
else
    echo "Unknown Argument. Please type isaac_sim_4.2.0"
    exit
fi

xhost +
echo "${dockerfile}"

docker build --build-arg ISAAC_SIM_VERSION=${isaac_sim_version} -t curobo_docker:${image_tag} -f ${dockerfile} ..


if [[ "$input_arg" == *isaac_sim* ]] ; then

    # 현재 사용자의 UID와 GID 가져오기
    USER_UID=$(id -u)
    USER_GID=$(id -g)
    docker run --rm --name automatica2025_container_$input_arg -it --gpus all -e "ACCEPT_EULA=Y" --network=host \
        --privileged \
        -e UID=$USER_UID -e GID=$USER_GID \
        -e "PRIVACY_CONSENT=Y" \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -e DISPLAY \
        -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
        -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
        -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
        -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
        -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
        -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
        -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
        -v ~/docker/isaac-sim/documents:/root/Documents:rw \
        -v ~/ros2_ws:/ros2_ws:rw \
        -v ~/bitbucket/curobo/example:/pkgs/curobo/examples/automatica2025:rw \
        --volume /dev:/dev \
         -v /tmp/.X11-unix:/tmp/.X11-unix \
        --gpus all --runtime=nvidia -e NVIDIA_DRIVER_CAPABILITIES=all \
        curobo_docker:$input_arg

else
    echo "Unknown docker"
fi
