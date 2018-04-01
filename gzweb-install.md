## Gzweb
Gzweb is installed on the server-side. Once the server is set up and running, clients can interact with the simulation simply by accessing the server's URL on a web browser.

## [Install](http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb)

1. prerequisite

```bash 
# download node v0.10.36
wget https://nodejs.org/dist/v0.10.36/node-v0.10.36.tar.gz

# install nodejs; change to node-v0.10.36 directory
./configure
make -j12
sudo make install
```

2. install gzweg

```bash
# download source 
hg clone https://bitbucket.org/osrf/gzweb
cd gzweb
hg up gzweb_1.3.0

# install gzweb
source /usr/share/gazebo/setup.sh
./deploy.sh -m 
```

## Run


## About
[Gzweb installation](http://gazebosim.org/tutorials?tut=gzweb_install&cat=gzweb)
