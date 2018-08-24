# specify the linux base image with your desired version ubuntu:<version>
FROM ubuntu:12.04

# Docker folder to work.
WORKDIR /app

COPY Tools/scripts/install-prereqs-ubuntu.sh /app

# Initial internal configuration.
RUN apt-get -y update
RUN apt-get -y install sudo iputils-ping lsb-core python-software-properties python-dev python-pip
RUN pip install -i https://pypi.python.org/simple/ --upgrade pip
RUN /app/Tools/scripts/install-prereqs-ubuntu.sh -y

CMD bash