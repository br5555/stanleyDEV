#!/usr/bin/env python

import urllib
import os
script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
rel_path = "../maps/white.png"
abs_file_path = os.path.join(script_dir, rel_path)
f = open(abs_file_path,'wb')
f = open('../maps/white.png','wb')
f.write(urllib.urlopen('https://cloud.githubusercontent.com/assets/7688443/24586308/714e066a-179d-11e7-8d92-ec2e08247e60.png').read())
f.close()


