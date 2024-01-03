FROM ros:noetic

# Install CURL, as we need it for debugging.
RUN apt-get -y update && apt-get install -y curl && rm -rf /var/lib/apt/lists/*

# Set the working directory to something sensible
WORKDIR /root/workdir

# Copy the setup.bash file over, alongside anything else you put in here and might want to use.
copy . .

# Load everything in the bashrc to make sure all commands are available and evoirement variables are set.
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/workdir/setup.bash' >> /root/.bashrc
