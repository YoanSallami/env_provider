#!/usr/bin/env python

import time
import sys
import copy
import rospy
import argparse
import underworlds
import numpy
from yaml import load
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from underworlds.types import Node, MESH, Situation
from underworlds.tools.loader import ModelLoader
from underworlds.helpers.transformations import *


class EnvProvider(object):
    def __init__(self, ctx, target_world, nodes_file_path, facts_file_path, mesh_dir):
        self.ctx = ctx
        self.target = ctx.worlds[target_world]
        self.target_world_name = target_world
        self.mesh_dir = mesh_dir
        self.geometric_description = None
        self.static_facts = None
        self.nodes_file_path = nodes_file_path
        self.facts_file_path = facts_file_path

        self.ros_pub = {"tf": rospy.Publisher("/tf", TFMessage, queue_size=10)}

    def read_geometric_description(self):
        rospy.loginfo("[env_provider] Loading static geometric description from : %s" % self.nodes_file_path)
        f = open(self.nodes_file_path, "r")
        self.geometric_description = load(f)

    def read_static_facts(self):
        rospy.loginfo("[env_provider] Loading static facts from : %s" % self.facts_file_path)
        f = open(self.facts_file_path, "r")
        self.static_facts = load(f)

    def load_nodes(self):
        nodes_to_update = []
        for static_node in self.geometric_description:
            node = Node(static_node["name"], MESH)
            if self.target.scene.nodebyname(static_node["name"]):
                node.id = self.target.scene.nodebyname(static_node["name"])[0].id

            if static_node["mesh"]:
                rospy.loginfo("[env_provider] Loading file : "+static_node["mesh"])
                try:
                    nodes_loaded = ModelLoader().load(self.mesh_dir+static_node["mesh"], self.ctx,
                                                  world=self.target_world_name, root=None, only_meshes=True,
                                                  scale=static_node["scale"])
                except Exception as e:
                    rospy.logwarn("[env_provider] Exception occurred with %s : %s" % (static_node["name"], str(e)))
                    continue
                for n in nodes_loaded:
                    if n.type == MESH:
                        node.properties["mesh_ids"] = n.properties["mesh_ids"]
                        node.properties["aabb"] = n.properties["aabb"]

            translation = identity_matrix()
            orientation = identity_matrix()
            if static_node["position"]:
                translation = translation_matrix([static_node["position"]["x"], static_node["position"]["y"],
                                                  static_node["position"]["z"]])
                if static_node["orientation"]:
                    orientation = euler_matrix(static_node["orientation"]["rx"], static_node["orientation"]["ry"],
                                               static_node["orientation"]["rz"], "rxyz")
            transformation = numpy.dot(translation, orientation)
            node.transformation = transformation
            nodes_to_update.append(node)

        if nodes_to_update:
            rospy.loginfo("[env_provider] Updating %s nodes to world <%s>..." % (str(len(nodes_to_update)),
                                                                                 self.target_world_name))
            self.target.scene.nodes.update(nodes_to_update)

    def publish_static_tf(self):
        for node in self.target.scene.nodes:
            if node != self.target.scene.rootnode and node.name != "map":
                t = TransformStamped()
                t.header.frame_id = "map"
                t.header.stamp = rospy.Time.now()

                t.child_frame_id = node.name
                position = translation_from_matrix(node.transformation)
                t.transform.translation.x = position[0]
                t.transform.translation.y = position[1]
                t.transform.translation.z = position[2]
                orientation = quaternion_from_matrix(node.transformation)
                t.transform.rotation.x = orientation[0]
                t.transform.rotation.y = orientation[1]
                t.transform.rotation.z = orientation[2]
                t.transform.rotation.w = orientation[3]

                tfm = TFMessage([t])
                self.ros_pub["tf"].publish(tfm)

    def load_facts(self):
        facts_to_update = []
        for static_fact in self.static_facts:
            sit = Situation(desc=static_fact)
            facts_to_update.append(sit)

        if facts_to_update:
            rospy.loginfo("[env_provider] Updating %s situations to world <%s>..." % (str(len(facts_to_update)),
                                                                                      self.target_world_name))
            self.target.timeline.update(facts_to_update)

    def run(self):
        self.read_geometric_description()
        self.read_static_facts()
        self.load_nodes()
        self.load_facts()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_static_tf()
            rate.sleep()


if __name__ == "__main__":
    sys.argv = [arg for arg in sys.argv if "__name" not in arg and "__log" not in arg]
    sys.argc = len(sys.argv)

    parser = argparse.ArgumentParser(description="Create a world with static objects described by a yaml file")
    parser.add_argument("world", help="Underworlds world to create")
    parser.add_argument("nodes_file_path", default="The path used to localize the static geometric description file")
    parser.add_argument("facts_file_path", default="The path used to localize the static facts description file")
    parser.add_argument("mesh_dir", help="The path to mesh directory")
    args = parser.parse_args()

    rospy.init_node("env_provider", anonymous=False)

    with underworlds.Context("Environment provider") as ctx:
        EnvProvider(ctx, args.world, args.nodes_file_path, args.facts_file_path, args.mesh_dir).run()
