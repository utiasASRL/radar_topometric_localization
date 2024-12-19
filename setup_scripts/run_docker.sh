# Assumes that ROOTDIR is set and pointing to radar_topometric_localization root directory
if [ $(docker inspect -f '{{.State.Running}}' radar_loc_$(whoami)) = "true" ]
then
	echo 'Container already running, joining it now.'
	docker exec -it radar_loc_$(whoami) bash
    cd $ROOTDIR
else
	docker volume create --driver local \
    --opt type=ext4 \
    --opt device=/dev/sdb1 \
    ssd_volume

	echo 'New container run initialized.'
	docker run -it --rm --name radar_loc_$(whoami) \
	--privileged \
	--network=host \
	-e DISPLAY=$DISPLAY \
	-e ROOTDIR=$ROOTDIR \
	-v ssd_volume:/ssd:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw \
	-v ${HOME}:${HOME}:rw \
	-v $ROOTDIR:$ROOTDIR:rw \
	-w $ROOTDIR radar_loc_$(whoami)
fi