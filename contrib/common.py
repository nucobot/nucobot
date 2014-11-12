import sys, os


def ensure_dir(f):
    if not os.path.exists(f):
        print "Created directory", f
        os.makedirs(f)

def exec_command(cmd_):
    command = ['bash', '-c', cmd_]
    proc = subprocess.Popen(command, stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    for line in proc.stdout:
      (key, _, value) = line.partition("=")
      os.environ[key] = value
    (stdout, stderr) = proc.communicate()
    if len(stderr) == 0:
        return 0
    print stderr
    return 1

def add_shortcut(name, icon, command):
    # The generation is done analogicaly to alacarte utilite
    if not os.path.isfile(icon):
        print "No such icon: " + icon
        exit(1)

    file_path = HOME_DIR + '/.local/share/applications/' + name + '_ad_gen.desktop'
    format_str = '#!/usr/bin/env xdg-open\n\n'\
                 '[Desktop Entry]\n'\
                 'Version=1.0\n'\
                 'Type=Application\n'\
                 'Terminal=true\n'\
                 'Name=' + name + '\n'\
                 'Icon=' + icon + '\n'\
                 'Exec=' + command
    try:
        desktop_file = open(file_path, 'w+')
        desktop_file.write(format_str)
        os.chmod(file_path, 0775)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print sys.exc_info()
        exit(1)

def gen_launcher(file_path, launcher_name, icon_name, command):
    format_str = '#!/bin/bash\n'
    format_str += '\n' + command + '\n\n'
    try:
        bash_scipt_file = open(file_path, 'w+')
        bash_scipt_file.write(format_str)
        os.chmod(file_path, 0744)
        print "Written to", file_path
    except:
        print "Error creating ", file_path
        print "Check the permissions"
        exit(1)
    add_shortcut(launcher_name, icon_name, file_path)




class BashRC_editor:
    def __init__(self, mark_, safety_ = true):
        self.safety_on = safety_
        self.marker = mark
        self.uid = os.getuid()
        if (self.uid == 0 and safety == true):
            raise RuntimeError('Unable to create a class while the uid = 0!')
        self.bashrc = os.path.expanduser("~") + '/.bashrc'
    
    def read(self):
        lines_with_poses = []
        to_writeback = []
        with open(self.bashrc, 'r') as fbashrc:
            for line in fbashrc:
                fpointer = 0
                if not self.marker in line:
                    to_writeback.append(line)
                    fpointer += 1
                else:
                    line.replace(self.marker, '')
                    lines_with_poses.append(fpointer, line)
        print 'Read %d lines:' % (len(lines_with_poses))
        print '%d: %s' % (a[0], a[1]) for a in lines_with_poses
        with open(self.bashrc, 'w') as fbashrc:
            fbashrc.writelines(to_writeback)
        return lines_with_poses

    def write(self, lines_with_poses):
        pass 

    def add_line(self, line, lines_with_poses, pose = 10000):
        return lines_with_poses.append([pose, line + self.marker])

