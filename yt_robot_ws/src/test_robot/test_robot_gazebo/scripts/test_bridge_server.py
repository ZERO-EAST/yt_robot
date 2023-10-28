# !/usr/bin/env python3
# -*- coding:utf-8 -*-
# @Time     : 2023/07/17 上午11:35
# @Author   : jiangyuandong
# @Email    : jyd_wy@163.com
# @File     : test_bridge_server.py
# @Software : PyCharm


import fnmatch
from rosbridge_library.capability import Capability
from rosbridge_library.internal.publishers import manager
from rosbridge_library.util import string_types


class Publish(Capability):
    publish_msg_fields = [(True, "topic", string_types)]
    topics_glob = None

    def __init__(self, protocol):
        # Call superclas constructor
        Capability.__init__(self, protocol)

        # Register the operations that this capability provides
        protocol.register_operation("publish", self.publish)

        # Save the topics that are published on for the purposes of unregistering
        self._published = {}

    def publish(self, message):
        # Do basic type checking
        self.basic_type_check(message, self.publish_msg_fields)
        topic = message["topic"]
        latch = message.get("latch", False)
        queue_size = message.get("queue_size", 100)

        if Publish.topics_glob is not None and Publish.topics_glob:
            self.protocol.log("debug", "Topic security glob enabled, checking topic: " + topic)
            match = False
        for glob in Publish.topics_glob:
            if fnmatch.fnmatch(topic, glob):
                self.protocol.log("debug", "Found match with glob " + glob + ", continuing publish...")
                match = True
                break
            if not match:
                self.protocol.log("warn", "No match found for topic, cancelling publish to: " + topic)
                return
        else:
            self.protocol.log("debug", "No topic security glob, not checking publish.")
        # Register as a publishing client, propagating any exceptions
        client_id = self.protocol.client_id
        manager.register(client_id, topic, latch=latch, queue_size=queue_size)
        self._published[topic] = True

        # Get the message if one was provided
        msg = message.get("msg", {})

        # Publish the message
        manager.publish(client_id, topic, msg, latch=latch, queue_size=queue_size)

    def finish(self):
        client_id = self.protocol.client_id
        for topic in self._published:
            manager.unregister(client_id, topic)
            self._published.clear()
