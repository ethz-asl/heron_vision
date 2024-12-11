#!/bin/bash

# Based on the ETH Robotics Summer school docker:
# https://github.com/ETHZ-RobotX/smb_docker/

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.

# Default options
DOCKER=heron_docker
DOCKERFILE=Dockerfile
NAME=heron
BUILD=false
WORKSPACE=/Users/$USER/code/heron_ws/
DATA=/Users/$USER/data/

help()
{
    echo "Usage: run_docker.sh
               [ -b | --build ] [ -n | --name <docker name> ]
               [ -h | --help  ] [ -w | --workspace </workspace/path> ]
               [ -d | --data </data/path> ] "
    exit 2
}

SHORT=bn:w:d:h
LONG=build,name:,workspace:,data:,help
OPTS=$(getopt -a -n run_docker --options $SHORT --longoptions $LONG -- "$@")
echo $OPTS

eval set -- "$OPTS"

while :
do
  case "$1" in
    -b | --build )
      BUILD="true"
      shift
      ;;
    -n | --name )
      NAME="$2"
      shift 2
      ;;
    -w | --workspace )
      WORKSPACE="$2"
      shift 2
      ;;
    -d | --data )
      DATA="$2"
      shift 2
      ;;
    -h | --help)
      help
      ;;
    --)
      shift;
      break
      ;;
    *)
      echo "Unexpected option: $1"
      help
      ;;
  esac
done


if [ "$BUILD" = true ]; then
    echo "Building docker: $DOCKERFILE as $DOCKER"
    docker build -f $DOCKERFILE --platform linux/arm64/v8 -t $DOCKER .
fi

#XAUTH=/tmp/.docker.xauth
#rm $XAUTH

#echo "Preparing Xauthority data..."
#xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
#if [ ! -f $XAUTH ]; then
#    if [ -n "$xauth_list" ]; then
#        echo $xauth_list | xauth -f $XAUTH nmerge -
#    else
#        touch $XAUTH
#    fi
#    chmod a+r $XAUTH
#fi

#echo "Done."
#echo ""
#echo "Verifying file contents:"
#file $XAUTH
#echo "--> It should say \"X11 Xauthority data\"."
##echo ""
#echo "Permissions:"
#ls -FAlh $XAUTH
#echo ""
echo "Running docker..."

docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume=$WORKSPACE:/root/heron_ws \
    --volume=$DATA:/root/data \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --net=host \
    --name=$NAME \
    ${DOCKER} \
    bash

echo "Done."