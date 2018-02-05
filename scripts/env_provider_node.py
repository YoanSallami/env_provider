#!/usr/bin/env python

import sys
import rospy
import argparse
import underworlds
from yaml import load
from underworlds.types import Node, MESH
from underworlds.tools.loader import ModelLoader
from underworlds.helpers.transformations import translation_matrix, euler_matrix, identity_matrix, compose_matrix


class EnvProviderNode(object):
    def __init__(self, ctx, target_world, file_path, mesh_dir):
        self.ctx = ctx
        self.target = ctx.worlds[target_world]
        self.target_world_name = target_world
        self.mesh_dir = mesh_dir

        self.read_description(file_path)
        self.load_nodes()

    def read_description(self, file_path):
        file = open(file_path, "r")
        self.yaml_file = load(file)

    def load_nodes(self):
        for static_node in self.yaml_file:
            node = Node(name=static_node["name"])
            if self.target.scene.nodes.nodebyname(static_node["name"]):
                node.id = self.target.scene.nodes.nodebyname(static_node["name"])[0].id
            if static_node["mesh"]:
                node.type = MESH
                if static_node["scale"]:
                    nodes_loaded = ModelLoader.load(static_node["mesh"], self.ctx, world=self.target_world_name, root=None, only_meshes=True, scale=static_node["scale"])
                else:
                    nodes_loaded = ModelLoader.load(static_node["mesh"], self.ctx, world=self.target_world_name, root=None, only_meshes=True)
                for n in nodes_loaded:
                    if n.type == MESH:
                        node.cad = n.cad
                        node.aabb = n.aabb
            position = identity_matrix()
            orientation = identity_matrix()
            if static_node["position"]:
                position = translation_matrix([static_node["position"]["x"], static_node["position"]["y"], static_node["position"]["z"]])
                if static_node["orientation"]:
                    orientation = euler_matrix([static_node["orientation"]["rx"],static_node["orientation"]["ry"],static_node["orientation"]["rz"]], "rxyz")
            transformation = compose_matrix(position, orientation)
            node.transformation = transformation




if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Create a world with static objects described by a yaml file")
    parser.add_argument("world", help="Underworlds world to create")
    parser.add_argument("file_path", default="The path used to localize the yaml file")
    parser.add_argument("mesh_dir", help="The path to mesh directory")
    args = parser.parse_args()

    rospy.init_node("env_manager", anonymous=False)

    with underworlds.Context("Environment provider") as ctx:
        EnvProviderNode(ctx, args.world, args.file_path, args.mesh_dir)