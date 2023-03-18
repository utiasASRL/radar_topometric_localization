# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
if [ $(docker inspect -f '{{.State.Running}}' radar_loc_$(whoami)) = "true" ]
then
	echo 'Container already running, joining it now.'
	docker exec -it radar_loc_$(whoami) bash
    cd $ROOTDIR
else
	echo 'New container run initialized.'
	docker run -it --rm --name radar_loc_$(whoami) \
	--privileged \
	--network=host \
	--gpus all \
	-e DISPLAY=$DISPLAY \
	-e ROOTDIR=$ROOTDIR \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw \
	-v ${HOME}:${HOME}:rw \
	-v $ROOTDIR:$ROOTDIR:rw \
	-w $ROOTDIR radar_loc_$(whoami)
fi