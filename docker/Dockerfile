FROM amd64/ubuntu:22.04

RUN apt-get update && apt-get install apt-utils sudo -y

# Create user
RUN useradd --create-home --shell=/bin/bash user
RUN chown -R user /home/user/
# Add the user to sudoers
RUN chmod -R o-w /etc/sudoers.d/
RUN usermod -aG sudo user
# Give the user a password
RUN echo user:4774 | chpasswd

USER root

#Install dependencies
RUN apt-get update \
    && apt-get install -y \
    openssh-client \
    git \
    python3-pip \
    libglx0 \
    mesa-utils \
    vim \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/user/workspace
RUN mkdir ~/.ssh && ssh-keyscan -t rsa github.com >> ~/.ssh/known_hosts

WORKDIR /home/user/workspace

RUN mkdir /home/user/workspace/chargedup

COPY . /home/user/workspace/chargedup

RUN mkdir -p /home/user/.ssh && ssh-keyscan -t rsa github.com >> /home/user/.ssh/known_hosts

RUN chown -R user:user /home/user

USER user

RUN cd /home/user/workspace/chargedup && \
    python3 -m pip install -r requirements-dev.txt

CMD /bin/bash
