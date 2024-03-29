#!/usr/bin/env python3

#  Copyright 2020 Unity Technologies
#  Edited by Matthew Woo 2022
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import sys
import json
import rospy
import socket
import logging
import threading
import importlib

from .tcp_sender import UnityTcpSender
from .client import ClientThread
from .subscriber import RosSubscriber
from .publisher import RosPublisher
from .service import RosService
from .unity_service import UnityService


class TcpServer:
    """
    Initializes ROS node and TCP server.
    """

    def __init__(self, node_name, buffer_size=1024, connections=2, tcp_ip="", tcp_port=-1):
        """
        Initializes ROS node and class variables.

        Args:
            node_name:               ROS node name for executing code
            buffer_size:             The read buffer size used when reading from a socket
            connections:             Max number of queued connections. See Python Socket documentation
        """
        if tcp_ip != "":
            self.tcp_ip = tcp_ip
        else:
            self.tcp_ip = rospy.get_param("/ROS_IP")

        if tcp_port != -1:
            self.tcp_port = tcp_port
        else:
            name = rospy.get_name()  
            prefix = rospy.get_param("PREFIX", "/server_endpoint")
            base = int(rospy.get_param("ROS_TCP_PORT", "10000"))

            # Since the prefix is "/server_endpoint_xxx"
            # /server_endpoint_ = 0 to 16
            # agent = 17 to 21
            # global = 17 to 22
            query = name[17:len(name)]
            id = 0
            global_pcl_base = 1000

            # Range 000-999 will be reserved for agents each agents can take 1 port
            if query[0:5] == "agent":
                if query[6:8] == "00":
                    id = int(query[8])
                elif query[6] == "0":
                    id = int(query[7:9])
                else:
                    id = int(query[6:9])   
                base += id
            
            # Range 1000-1100 will be reserved for sensors
            # Range 1000-1004 will be reserved for global point cloud
            if query[0:6] == "global":
                if query[7:10] == "pcl":
                    id = int(query[11])
                    base += global_pcl_base + id

            #Range 1005-1024 will be reserved for external cameras
            
            print('node ['+ name + '] on port ' + str(base))
            self.tcp_port = base

        self.unity_tcp_sender = UnityTcpSender()

        self.node_name = node_name
        self.publishers = {}
        self.subscribers = {}
        self.ros_services = {}
        self.unity_services = {}
        self.buffer_size = buffer_size
        self.connections = connections
        self.syscommands = SysCommands(self)
        self.pending_srv_id = None
        self.pending_srv_is_request = False

    def start(self, publishers=None, subscribers=None):
        if publishers is not None:
            self.publishers = publishers
        if subscribers is not None:
            self.subscribers = subscribers
        server_thread = threading.Thread(target=self.listen_loop)
        # Exit the server thread when the main thread terminates
        server_thread.daemon = True
        server_thread.start()

    def listen_loop(self):
        """
        Creates and binds sockets using TCP variables then listens for incoming connections.
        For each new connection a client thread will be created to handle communication.
        """
        rospy.loginfo("Starting server on {}:{}".format(self.tcp_ip, self.tcp_port))
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((self.tcp_ip, self.tcp_port))

        while True:
            tcp_server.listen(self.connections)

            try:
                (conn, (ip, port)) = tcp_server.accept()
                ClientThread(conn, self, ip, port).start()
            except socket.timeout as err:
                logging.exception("ros_tcp_endpoint.TcpServer: socket timeout")

    def send_unity_error(self, error):
        self.unity_tcp_sender.send_unity_error(error)

    def send_unity_message(self, topic, message):
        self.unity_tcp_sender.send_unity_message(topic, message)

    def send_unity_service(self, topic, service_class, request):
        return self.unity_tcp_sender.send_unity_service_request(topic, service_class, request)

    def send_unity_service_response(self, srv_id, data):
        self.unity_tcp_sender.send_unity_service_response(srv_id, data)

    def handle_syscommand(self, topic, data):
        function = getattr(self.syscommands, topic[2:])
        if function is None:
            self.send_unity_error("Don't understand SysCommand.'{}'".format(topic))
            return
        else:
            message_json = data.decode("utf-8")
            params = json.loads(message_json)
            function(**params)


class SysCommands:
    def __init__(self, tcp_server):
        self.tcp_server = tcp_server

    def subscribe(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't subscribe to a blank topic name! SysCommand.subscribe({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.subscribe - Unknown message class '{}'".format(message_name)
            )
            return

        rospy.loginfo("RegisterSubscriber({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.subscribers:
            self.tcp_server.subscribers[topic].unregister()

        self.tcp_server.subscribers[topic] = RosSubscriber(topic, message_class, self.tcp_server)

    def publish(self, topic, message_name, queue_size=10, latch=False):
        if topic == "":
            self.tcp_server.send_unity_error(
                "Can't publish to a blank topic name! SysCommand.publish({}, {})".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name)
        if message_class is None:
            self.tcp_server.send_unity_error(
                "SysCommand.publish - Unknown message class '{}'".format(message_name)
            )
            return

        rospy.loginfo("RegisterPublisher({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.publishers:
            self.tcp_server.publishers[topic].unregister()

        self.tcp_server.publishers[topic] = RosPublisher(topic, message_class, queue_size, latch)

    def ros_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterRosService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        rospy.loginfo("RegisterRosService({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.ros_services:
            self.tcp_server.ros_services[topic].unregister()

        self.tcp_server.ros_services[topic] = RosService(topic, message_class)

    def unity_service(self, topic, message_name):
        if topic == "":
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Can't register a blank topic name!".format(
                    topic, message_name
                )
            )
            return

        message_class = resolve_message_name(message_name, "srv")
        if message_class is None:
            self.tcp_server.send_unity_error(
                "RegisterUnityService({}, {}) - Unknown service class '{}'".format(
                    topic, message_name, message_name
                )
            )
            return

        rospy.loginfo("RegisterUnityService({}, {}) OK".format(topic, message_class))

        if topic in self.tcp_server.unity_services:
            self.tcp_server.unity_services[topic].unregister()

        self.tcp_server.unity_services[topic] = UnityService(topic, message_class, self.tcp_server)

    def response(self, srv_id):  # the next message is a service response
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = False

    def request(self, srv_id):  # the next message is a service request
        self.tcp_server.pending_srv_id = srv_id
        self.tcp_server.pending_srv_is_request = True

    def topic_list(self):
        self.tcp_server.unity_tcp_sender.send_topic_list()


def resolve_message_name(name, extension="msg"):
    try:
        names = name.split("/")
        module_name = names[0]
        class_name = names[1]
        importlib.import_module(module_name + "." + extension)
        module = sys.modules[module_name]
        if module is None:
            rospy.logerr("Failed to resolve module {}".format(module_name))
        module = getattr(module, extension)
        if module is None:
            rospy.logerr("Failed to resolve module {}.{}".format(module_name, extension))
        module = getattr(module, class_name)
        if module is None:
            rospy.logerr(
                "Failed to resolve module {}.{}.{}".format(module_name, extension, class_name)
            )
        return module
    except (IndexError, KeyError, AttributeError, ImportError) as e:
        rospy.logerr("Failed to resolve message name: {}".format(e))
        return None
