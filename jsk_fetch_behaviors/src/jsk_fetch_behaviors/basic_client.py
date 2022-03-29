# -*- coding: utf-8 -*-

from behavior_manager_server.base_client import BaseClient
from sound_play.libsoundplay import SoundClient

import rospy


class FetchBasicClient(BaseClient):

    def __init__(self, client_name):

        super(FetchBasicClient, self).__init__(client_name)
        self.sound_client = SoundClient()

    def say(self, text, silent_mode=False, blocking=True):

        rospy.loginfo('Said: {}'.format(text))
        if not silent_mode:
            self.sound_client.say(text, blocking=blocking)
