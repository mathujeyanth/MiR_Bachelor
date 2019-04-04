import tensorflow as tf
from tensorflow.python.platform import gfile


def load_pb(path_to_pb):
    with tf.gfile.GFile(path_to_pb, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
    with tf.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name='')
        return graph


class Predictions:
    def __init__(self, GRAPH_PB_PATH):
        self.model = load_pb(GRAPH_PB_PATH)
        self.outputs = self.model.get_tensor_by_name('action:0')
        self.inputs = self.model.get_tensor_by_name('vector_observation:0')
        self.eps = self.model.get_tensor_by_name('epsilon:0')
        self.sess = tf.Session(graph=self.model)
        print("Input size: ", self.inputs[0].shape[0])
        print("Output size: ", self.outputs[0].shape[0])
        print("Epsilon size: ", self.eps[0].shape[0])

    def getPrediction(self, epsilonValues, inputValues):
        y_out = self.sess.run(self.outputs, feed_dict={self.eps: epsilonValues, self.inputs: inputValues})
        print(y_out)
        return y_out
