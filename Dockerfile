FROM dragon7/carnd-kidnapped-vehicle-project

# Install CarND-Path-Planning-Project
WORKDIR /root/workspace
COPY ./ CarND-Path-Planning-Project/

# Create Eclipse IDE project
WORKDIR /root/workspace/CarND-Path-Planning-Project
RUN mkdir build
RUN mv CMakeLists.txt src
RUN sed -i 's/src/./g' src/CMakeLists.txt
WORKDIR /root/workspace/CarND-Path-Planning-Project/build
RUN cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src/
RUN make

WORKDIR /root/workspace
