FROM ubuntu:18.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y git sudo g++ cmake 
# Necessary to avoid blocking installation of tzdata
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -y install tzdata

# Install GTest, necessary for CPPAD
RUN apt-get install -y libgtest-dev 
RUN cd /usr/src/gtest && mkdir build && cd build && cmake .. && make install


# control-toolbox
# Copy directory content to app
RUN mkdir /control-toolbox
WORKDIR /control-toolbox
COPY . /control-toolbox
RUN cd ct && ./install_deps.sh
RUN cd ct && ./install_hpipm.sh

# Put all the library sources in libsrc
RUN mkdir /libsrc 
WORKDIR /libsrc 

# kindr, necessary for control-toolbox
RUN git clone https://github.com/ANYbotics/kindr.git && cd kindr && git checkout master
RUN cd kindr && mkdir build && cd build && cmake .. && make install

# Install the control-toolbox
WORKDIR /control-toolbox

# Dev packages
RUN chmod -R 755 .devcontainer/
RUN .devcontainer/install_dev_pkg.sh

# Set workdir
WORKDIR /control-toolbox
