#!/usr/bin/env python

import time
import sys
import rospy
import argparse
import underworlds
import numpy
from yaml import load
from underworlds.types import Node, MESH
from underworlds.tools.loader import ModelLoader
from underworlds.helpers.transformations import translation_matrix, euler_matrix, identity_matrix, compose_matrix


class EnvProvider(object):
    def __init__(self, ctx, target_world, file_path, mesh_dir):
        self.ctx = ctx
        self.target = ctx.worlds[target_world]
        self.target_world_name = target_world
        self.mesh_dir = mesh_dir

        self.read_description(file_path)
        self.load_nodes()

    def read_description(self, file_path):
        rospy.loginfo("[env_provider] Loading static description from : %s" % file_path)
        file = open(file_path, "r")
        self.yaml_file = load(file)

    def load_nodes(self):
        nodes_to_update = []
        for static_node in self.yaml_file:
            node = Node(static_node["name"], MESH)
            if self.target.scene.nodebyname(static_node["name"]):
                node.id = self.target.scene.nodebyname(static_node["name"])[0].id

            if static_node["mesh"]:
                nodes_loaded = ModelLoader().load(self.mesh_dir+static_node["mesh"], self.ctx, world=self.target_world_name, root=None, only_meshes=True, scale=static_node["scale"])
                for n in nodes_loaded:
                    if n.type == MESH:
                        node.properties["mesh_ids"] = n.properties["mesh_ids"]

            translation = identity_matrix()
            orientation = identity_matrix()
            if static_node["position"]:
                translation = translation_matrix([static_node["position"]["x"], static_node["position"]["y"], static_node["position"]["z"]])
                if static_node["orientation"]:
                    orientation = euler_matrix(static_node["orientation"]["rx"], static_node["orientation"]["ry"], static_node["orientation"]["rz"], "rxyz")
            transformation = numpy.dot(translation, orientation)
            node.transformation = transformation
            nodes_to_update.append(node)

        if nodes_to_update:
            rospy.loginfo("[env_provider] Updating %s nodes to world <%s>..." % (str(len(nodes_to_update)), self.target_world_name))
            self.target.scene.nodes.update(nodes_to_update)

        rospy.spin()


if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Create a world with static objects described by a yaml file")
    parser.add_argument("world", help="Underworlds world to create")
    parser.add_argument("file_path", default="The path used to localize the yaml file")
    parser.add_argument("mesh_dir", help="The path to mesh directory")
    args = parser.parse_args()

    rospy.init_node("env_provider", anonymous=False)

    with underworlds.Context("Environment provider") as ctx:
        EnvProvider(ctx, args.world, args.file_path, args.mesh_dir)