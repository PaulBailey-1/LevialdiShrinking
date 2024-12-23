
#include <cuda_runtime.h>

#include "Levialdi.h"

__global__ void levialdiShrinkingOperator(bool* input, bool* output, int height) {
    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int index = height * col + row;

    if (input[index]) { // Case 1
        output[index] = input[index - 1] || input[index - height] || input[index - height - 1];
    }
    else { // Case 2
        output[index] = input[index - 1] && input[index - height];
    }
}

__global__ void getComponentCorners(bool* input, bool* output, int height, bool* notDone, int* corners, int* cornersNum) {

    int col = blockIdx.x * blockDim.x + threadIdx.x;
    int row = blockIdx.y * blockDim.y + threadIdx.y;
    int index = height * col + row;

    if (output[index]) {
        *notDone = true;
    } else if (input[index] && !output[index + 1] && !output[index + height] && !output[index + height + 1]) {
        // This needs to be mutex protected!
        corners[*cornersNum * 2] = col;
        corners[*cornersNum * 2 + 1] = row;
        atomicAdd(cornersNum, 1);
        // This could overflow the memory! To bad it can't be dynamic...
    }
}

std::vector<Corner> levialdiShrink(const BinaryArray& input) {

    int height = input.rows();
    int size = input.rows() * input.cols();
    bool* buffers[2];

    cudaError_t err = cudaSuccess;
    err = cudaMalloc((void**) &buffers[0], size);
    if (err == cudaSuccess) {
        err = cudaMalloc((void**)&buffers[1], size);
    }
    if (err != cudaSuccess) {
        fprintf(stderr, "Failed to allocate device buffers (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
    err = cudaMemcpy(buffers[0], input.data(), size, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        fprintf(stderr, "Failed to copy to device buffers (error code %s)!\n", cudaGetErrorString(err));
        exit(EXIT_FAILURE);
    }
       
    dim3 threadsPerBlock(10, 10);
    dim3 numBlocks(input.cols() / threadsPerBlock.x, input.rows() / threadsPerBlock.y);
    int step = 0;

    std::vector<Corner> corners;
    bool* notDone = nullptr;
    int* cornersBuffer = nullptr;
    int* cornersNum = nullptr;
    cudaMallocManaged(&notDone, sizeof(bool));
    cudaMallocManaged(&cornersBuffer, sizeof(int) * 10);
    cudaMallocManaged(&cornersNum, sizeof(int));
    *notDone = true;

    while (*notDone) {
        bool* input = buffers[step % 2];
        bool* output = buffers[(step + 1) % 2];
        levialdiShrinkingOperator<<<numBlocks, threadsPerBlock>>>(input, output, height);
        cudaDeviceSynchronize();

        err = cudaGetLastError();
        if (err != cudaSuccess) {
            fprintf(stderr, "Failed to launch levialdiShrinkingOperator kernel (error code %s)!\n",
                cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }

        *notDone = false;
        *cornersNum = 0;
        getComponentCorners<<<numBlocks, threadsPerBlock>>>(input, output, height, notDone, cornersBuffer, cornersNum);
        cudaDeviceSynchronize();

        err = cudaGetLastError();
        if (err != cudaSuccess) {
            fprintf(stderr, "Failed to launch getComponentCorners kernel (error code %s)!\n",
                cudaGetErrorString(err));
            exit(EXIT_FAILURE);
        }
        
        for (int i = 0; i < *cornersNum; i++) {
            corners.push_back(Corner(cv::Point(cornersBuffer[i * 2], cornersBuffer[i * 2 + 1]), step));
        }

        step++;
    }


    cudaFree(buffers[0]);
    cudaFree(buffers[1]);
    cudaFree(notDone);
    cudaFree(cornersBuffer);
    cudaFree(cornersNum);

    return corners;
}