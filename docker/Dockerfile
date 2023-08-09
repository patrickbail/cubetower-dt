FROM nvcr.io/nvidia/isaac-sim:2022.2.1

# Run the commands to install additional requirements
RUN ./python.sh -m pip install open3d
COPY requirements.txt /tmp/requirements.txt
RUN ./python.sh -m pip install -r /tmp/requirements.txt
