#!/usr/bin/python
# Author: Anton Mitrokhin, MIPT 2014

import sys, os, platform, shutil


def add_eclipse_launcher(INSTALL_DIR):
    file_path = os.path.expanduser("~") + '/.local/share/applications/eclipse.desktop'
    format_str = '[Desktop Entry]\n'\
                 'Name=Eclipse\n'\
                 'Type=Application\n'\
                 'Exec='+INSTALL_DIR+'/eclipse/eclipse\n'\
                 'Terminal=false\n'\
                 'Icon=/opt/eclipse/icon.xpm\n'\
                 'Comment=Integrated Development Environment\n'\
                 'NoDisplay=false\n'\
                 'Categories=Development;IDE\n'\
                 'Name[en]=Eclipse\n'\
                 'Exec=env UBUNTU_MENUPROXY=0 '+INSTALL_DIR+'/eclipse/eclipse'
    try:
        desktop_file = open(file_path, 'w+')
        desktop_file.write(format_str)
        os.chmod(file_path, 0775)
        print "Successfully written to", file_path
    except:
        print "Error creating ", file_path
        print sys.exc_info()
        exit(1)



from optparse import OptionParser
from optparse import OptionGroup
if __name__ == '__main__':
    class MyParser(OptionParser):
        def format_epilog(self, formatter):
            return self.epilog

    default_url = "http://www.eclipse.org/downloads/download.php?file=/technology/epp/downloads/release/luna/SR1/eclipse-cpp-luna-SR1-linux-gtk-x86_64.tar.gz&mirror_id=1029"

    examples =  ("This script helps to install eclipse editor onto your system\n" + 
    "Only Ubuntu 14.04 is currently supported as target operating system\n"+ 
    "The program must be run as root!\n\nExamples:\n" +
    "Load eclipse to the current directory\n\teclipse_deploy.py -l\n" +
    "Load and install in '/opt/eclipse' with specified eclipse.ini file\n\teclipse_deploy.py -l -i --configure='./eclipse_deploy/eclipse.ini'\n")

    parser = MyParser(usage="Usage: sudo eclipse_deploy.py -l/-i [options]", epilog=examples)
    parser.add_option('-l', '--load-eclipse', dest='load_eclipse',
        help='ask to download eclipse into the current directory', default=False, action="store_true")
    parser.add_option('-i', '--install', dest='install_eclipse',
        help='ask to install the eclipse', default=False, action="store_true")
    parser.add_option('-v', '--version', dest='print_version',
        help='ask to print out system information', default=False, action="store_true")
    download_group = OptionGroup(parser, "Options for downloading the eclipse",
                    "These options must be used with -l option.")
    download_group.add_option('--url', dest='eclipse_download_url',
        help='specify where the eclipse should be downloaded from. Default: <Tomsk State University>', default=default_url)
    download_group.add_option('--download-path', dest='eclipse_download_path',
        help='specify the location eclipse should be downloaded to. Default: ./', default="./")
    parser.add_option_group(download_group)
    install_group = OptionGroup(parser, "Options passed to eclipse installer",
                    "These options must be used with -i option.")
    install_group.add_option('--eclipse-folder', dest='eclipse_folder',
        help='set the path containing downloaded <eclipse> folder. Default: <the one specified with --path>', default="./")
    install_group.add_option('--install-location', dest='install_location',
        help='set the installation path. Default: /opt', default="/opt")
    install_group.add_option('--configure', dest='eclipse_configure',
        help='overwrite the default eclipse.ini with the specified file', default="")
    parser.add_option_group(install_group)
    (options, args) = parser.parse_args()

    if not (options.print_version or options.load_eclipse or options.install_eclipse):
        parser.print_help()

    if (options.print_version):
        print 'Network name: %s' % (platform.node())
        print 'Python version: %s %s compiled with %s' % (platform.python_implementation(), platform.python_version(), platform.python_compiler())
        print 'System: %s %s (%s)' % (platform.system(), platform.release(), platform.machine())
        print 'Platform version: %s' % (platform.version())


    if (options.load_eclipse or options.install_eclipse):
        if (not '#66-Ubuntu SMP' in platform.version()):
            print 'This platform is not supported, sorry!'
            exit(0)
        if os.getuid() != 0:
    	    print "This program is intended to run as root (type eclipse_deploy.py -h for help)"
    	    exit(1)

    if (options.eclipse_download_path != './' and options.eclipse_folder == './'):
        options.eclipse_folder = options.eclipse_download_path

    if (options.load_eclipse):
        print "-l is not yet implemented!"

    if (options.install_eclipse):
        try:
            shutil.move(options.eclipse_folder + 'eclipse', options.install_location)
        except:
            print "The 'eclipse' folder is expected to be in the directory '%s'!" % (options.eclipse_folder)
            exit(1)
        add_eclipse_launcher(options.install_location)

        if (options.eclipse_configure != ''):
            print "--configure is not yet implemented!"


