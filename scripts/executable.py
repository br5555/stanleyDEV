#!/usr/bin/env python
import os
import stat

script_dir = os.path.dirname(__file__)
rel_path1 = "executable.py"
abs_file_path1 = os.path.join(script_dir, rel_path1)
rel_path2 = "path_linear.py"
abs_file_path2 = os.path.join(script_dir, rel_path2)
rel_path3 = "my.py"
abs_file_path3 = os.path.join(script_dir, rel_path3)
rel_path4 = "path_eight_polar.py"
abs_file_path4 = os.path.join(script_dir, rel_path4)
rel_path5 = "path_circle_polar.py"
abs_file_path5 = os.path.join(script_dir, rel_path5)
rel_path6 = "stanley2.py"
abs_file_path6 = os.path.join(script_dir, rel_path6)

st = os.stat(abs_file_path1)
os.chmod(abs_file_path1, st.st_mode | stat.S_IEXEC)
st = os.stat(abs_file_path2)
os.chmod(abs_file_path2, st.st_mode | stat.S_IEXEC)
st = os.stat(abs_file_path3)
os.chmod(abs_file_path3, st.st_mode | stat.S_IEXEC)
st = os.stat(abs_file_path4)
os.chmod(abs_file_path4, st.st_mode | stat.S_IEXEC)
st = os.stat(abs_file_path5)
os.chmod(abs_file_path5, st.st_mode | stat.S_IEXEC)
st = os.stat(abs_file_path6)
os.chmod(abs_file_path6, st.st_mode | stat.S_IEXEC)

