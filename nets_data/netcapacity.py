#!/usr/bin/env python2
import os, sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

from optparse import OptionParser
from xml import sax

# Usage message
usage="""{0} [options] --net-file FILE

Reads a SUMO network file and estimates the number of vehicles
that fit in it. This estimation is very optimistic, assuming no
gap between vehicles.
""".format(sys.argv[0]).strip()

# Imports sumolib or warns about its absence
try:
    from sumolib.net import NetReader
except ImportError:
    print """Error importing package 'sumolib'.

This script depends on SUMO's library for parsing xml files in
python, which is distributed with SUMO on tools/sumolib (see
http://sumo.sourceforge.net).

Please put this package on the same directory
as this script, or somewhere in the python path
(e.g. ~/.local/lib/python2.7/site-packages).
"""
    exit(1);


def parseArgs(argv):
    """Parse the command line arguments and return (options, args).
    """

    # Registers the options
    parser = OptionParser(usage=usage)
    parser.add_option('-n', '--net-file', metavar='FILE', dest='net_file',
                      help='A SUMO network whose capacity will be found.')
    parser.add_option('-l', '--vehicle-length', metavar='N',
                      dest='veh_length', type='float', default=5.0,
                      help='The average length of vehicles [default: %default]')

    # Parses and verifies the result
    (options, args) = parser.parse_args(argv)
    if options.net_file is None:
        parser.error("Option --net-file is mandatory and wasn't supplied")

    return (options, args)

def main():
    # Parse command line options
    (options, args) = parseArgs(sys.argv)

    # Initialize the parser
    handler = NetReader()
    parser = sax.make_parser()
    parser.setContentHandler(handler)

    # Parse the file
    print 'Parsing SUMO net file "%s"...' % options.net_file
    try:
        parser.parse(options.net_file)
    except IOError as e:
        print 'Error reading net file:', e.strerror
        sys.exit(1)

    # Obtain the results
    edges = handler.getNet().getEdges()
    total_length = sum( e.getLength() for e in edges )
    total_length_lanes = sum( e.getLength() * e.getLaneNumber() for e in edges )
    capacity = float(total_length) / options.veh_length
    capacity_lanes = float(total_length_lanes) / options.veh_length
    # Show the results
    print 'Number of edges:', len(edges)
    print 'Total network length (disregarding #lanes): %.2f meters.' % total_length
    print 'Estimated network capacity (disregarding #lanes): %.2f vehicles.' % capacity
    print 'Total network length (considering #lanes): %.2f meters.' % total_length_lanes
    print 'Estimated network capacity (considering #lanes): %.2f vehicles.' % capacity_lanes
    

if __name__ == '__main__': main()
