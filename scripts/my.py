#!/usr/bin/env python

import urllib
f = open('../maps/white.png','wb')
f.write(urllib.urlopen('https://cloud.githubusercontent.com/assets/7688443/24586308/714e066a-179d-11e7-8d92-ec2e08247e60.png').read())
f.close()


