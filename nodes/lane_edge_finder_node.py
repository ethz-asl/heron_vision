#!/usr/bin/env python

from heron_vision import LaneEdgeFinder

if __name__ == '__main__':
    node = LaneEdgeFinder("iccs_lane_edge_finder")
    node.start_spin()
