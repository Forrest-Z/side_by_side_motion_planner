#! /usr/bin/env python
"""Collect command-line options in a dictionary"""


def args(argv):
    out_str=''
    for s in argv:
        out_str+=s+" "
    return out_str

if __name__ == '__main__':
    from sys import argv
    import subprocess
    subprocess.call("source ../devel/setup.bash",shell=True)
    command="roslaunch vehicle1.launch " 
    command+="use_simulator:=false use_safety:=false robot:=min_bot "#the white space at the end of this line is necessary
    command+=args(argv[1:])
    subprocess.call(command,shell=True)
    
