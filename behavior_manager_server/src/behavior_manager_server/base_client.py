# -*- coding: utf-8 -*-

class BaseClient(object):

    def __init__(self, client_name):

        self.client_name = client_name


class SimpleClient(object):

    def __init__(self, client_name):

        super(SimpleClient, self).__init__(client_name)
