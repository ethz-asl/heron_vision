#!/usr/bin/env python

from heron_vision import CrackFinder

if __name__ == '__main__':
    node = CrackFinder("iccs_crack_finder")
    node.start_spin()
