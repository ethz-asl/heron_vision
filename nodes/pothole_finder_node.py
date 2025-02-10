#!/usr/bin/env python

from heron_vision import PotholeFinder

if __name__ == '__main__':
    node = PotholeFinder("iccs_pothole_finder")
    node.start_spin()
