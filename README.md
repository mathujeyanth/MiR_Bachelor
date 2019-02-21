# MiR_Bachelor

# QoL commands
Unity

train: mlagents-learn config/trainer_config.yaml --run-id=firstRun --train

tensorboard: tensorboard --logdir=summaries

# Funktion

float getTargetAngle(Vector2 point)
    {
        float anAngle = Vector2.SignedAngle(new Vector2(0f, 1f), point - (Vector2)rb.transform.localPosition);
        float rotation = rb.rotation % 360;
        rotation = anAngle - rotation;

        if (rotation > 180)
            rotation = rotation - 360;
        else if (rotation < -180)
            rotation = rotation + 360;

        return rotation;
    }
