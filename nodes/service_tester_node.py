#!/usr/bin/env python

from heron_vision import ServiceTester

if __name__ == '__main__':
    node = ServiceTester("iccs_service_tester")

    node.start_spin()
