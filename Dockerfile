# specify the linux base image with your desired version ubuntu:<version>
FROM ubuntu:12.04

# Docker folder to work.
WORKDIR /app

# Initial internal configuration.
RUN apt-get -y update
RUN apt-get -y install sudo iputils-ping lsb-core python-software-properties
#software-properties-common
#RUN apt-get -y install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
#RUN apt-get -y install build-essential ccache g++ gawk git make wget
#RUN apt-get -y install git libtool libxml2-dev libxslt1-dev python-dev python-pip python-setuptools python-matplotlib python-serial python-scipy python-opencv python-numpy python-pyparsing realpath

#RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo
#RUN adduser root sudo
#USER docker

CMD bash