# -*- coding: utf-8 -*-


def load_client_class(class_string):
    package_name = class_string.split('.')[0]
    module_name = class_string.split('.')[1]
    class_name = class_string.split('.')[2]
    client_class = getattr(
        getattr(__import__(package_name), module_name), class_name)
    return client_class


class BaseClient(object):

    def __init__(self, client_name):

        self.client_name = client_name


class SimpleClient(BaseClient):

    def __init__(self, client_name):

        super(SimpleClient, self).__init__(client_name)
