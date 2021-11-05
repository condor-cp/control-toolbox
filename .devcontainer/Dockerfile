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

# Use non root user to avoid ownership problems for the user
# See https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user 
ARG USERNAME=devuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME