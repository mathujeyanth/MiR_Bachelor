# MiR_Bachelor

# QoL commands
Unity

train: mlagents-learn config/trainer_config.yaml --run-id=firstRun --train

imitate: mlagents-learn config/online_bc_config.yaml --run-id=ImitateMe --train --slow

tensorboard: tensorboard --logdir=summaries
