# MiR_Bachelor

# QoL commands
Unity

train: mlagents-learn config/trainer_config.yaml --env=<some build> --run-id=<some name> --train
    Extra: "--load"

imitate: mlagents-learn config/online_bc_config.yaml --run-id=<some name> --train --slow
    Extra: "--load"

tensorboard: tensorboard --logdir=<some dir>

docker run --name dockerMir --mount type=bind,source="$(pwd)"/unity-volume,target=/unity-volume -p 5005:5005 docker_1:latest First_run --docker-target-name=unity-volume trainer_config.yaml --env=Dockerv-01 --train --run-id=docker_first_trial

#The following command exits with code (0)
docker run --name fuckthis8 -v type=bind,source=ml-agents-master/unity-volume,target=/unity-volume -p 5005:5005 shit:latest 3DBall mlagents-learn --help
