if [ $(docker inspect -f '{{.State.Running}}' vtr3_masking_daniil) = "true" ]
then
	echo 'Container already running, joining it now.'
	docker exec -it vtr3_masking_daniil bash
    cd /home/dli/masking_project
else
	echo 'New container run initialized.'
	docker run -it --rm --name vtr3_masking_daniil \
	--privileged \
	--network=host \
	--gpus all \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ${HOME}/.Xauthority:${HOME}/.Xauthority:rw \
	-v ${HOME}:${HOME}:rw \
	-v /raid/krb/boreas:/raid/krb/boreas \
	-v /raid/dli:/raid/dli:rw \
	-v ${HOME}/masking_project:${HOME}/masking_project:rw vtr3_masking_daniil
fi