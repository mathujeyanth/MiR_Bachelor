# MiR_Bachelor

# QoL commands
Unity

train: mlagents-learn config/trainer_config.yaml --env="some build" --run-id="some name" --train
    
   Extra: "--load"

imitate: mlagents-learn config/online_bc_config.yaml --run-id="some name" --train --slow
    
   Extra: "--load"

tensorboard: tensorboard --logdir="some dir"

docker run --name "docker name" --mount type=bind,source="$(pwd)"/unity-volume,target=/unity-volume -p 5005:5005 "docker_img":latest "some name" --docker-target-name=unity-volume trainer_config.yaml --env=Dockerv-01 --train --run-id=docker_first_trial

#The following command exits with code (0)
docker run --name "docker name" -v type=bind,source=ml-agents-master/unity-volume,target=/unity-volume -p 5005:5005 "docker_img":latest "some name" mlagents-learn --help
