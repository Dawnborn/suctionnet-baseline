# conda activate
CUDA_VERSION=10.2 # not work
CUDA_VERSION=11.1
# CUDA_VERSION=10.1

export CUDA_HOME=/data/hdd1/storage/junpeng/ws_graspnerf/mycudatoolkit/cuda-$CUDA_VERSION

export PATH=/data/hdd1/storage/junpeng/ws_graspnerf/mycudatoolkit/cuda-${CUDA_VERSION}/bin:$PATH

export LD_LIBRARY_PATH=/data/hdd1/storage/junpeng/ws_graspnerf/mycudatoolkit/cuda-${CUDA_VERSION}/lib64:$LD_LIBRARY_PATH