#!/usr/bin/python
# Author: Anton Mitrokhin, MIPT 2014
import sys, os, platform, shutil


from optparse import OptionParser
from optparse import OptionGroup
if __name__ == '__main__':
    class MyParser(OptionParser):
        def format_epilog(self, formatter):
            return self.epilog

    default_url = "https://github.com/Itseez/opencv/archive/3.0.0-beta.zip"

    examples =  ("This script helps to download and build opencv 3.0.0\n" + 
    "Only Ubuntu 14.04 is currently supported as target operating system\n"+ 
    "\n\nExamples:\n")

    parser = MyParser(usage="Usage: opencv_deploy.py -l/-i [options]", epilog=examples)
    parser.add_option('-l', '--load', dest='load_opencv',
        help='ask to download opencv into the current directory', default=False, action="store_true")
    parser.add_option('-b', '--build', dest='build_opencv',
        help='ask to build opencv', default=False, action="store_true")
    parser.add_option('-i', '--install', dest='install_opencv',
        help='ask to install opencv', default=False, action="store_true")

    download_group = OptionGroup(parser, "Options for downloading the opencv",
                    "These options must be used with -l option.")
    parser.add_option_group(download_group)

    build_group = OptionGroup(parser, "Options for opencv installation",
                    "These options must be used with -b option.")
    parser.add_option_group(build_group)

    install_group = OptionGroup(parser, "Options for opencv installation",
                    "These options must be used with -i option.")
    parser.add_option_group(install_group)

    (options, args) = parser.parse_args()

    if not (options.load_opencv or options.build_opencv or options.install_opencv):
        parser.print_help()

    if (not '#66-Ubuntu SMP' in platform.version()):
        print 'This platform is not supported, sorry!'
        exit(0)

    print "To implement ..."

